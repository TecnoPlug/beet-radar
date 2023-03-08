import json
import logging
import threading
import socket
import select
import requests
import time

sshDisconnected = False


def checkRemoteHost(server_port, addr):
    global sshDisconnected
    logging.info("Starting requests")
    while True:
        time.sleep(30)
        try:
            logging.info(f"Requesting http://168.197.51.94:{server_port}")
            mres = requests.post(
                f"http://168.197.51.94:{server_port}/ping",
                json={'mac': addr}
            )
            mjres = json.loads(mres.text)
            logging.debug(mjres)
            if mjres['sent'] != 'OK':
                logging.critical("Erro requesting ping")
                sshDisconnected = True
                return
            mres = requests.post(
                f"http://168.197.51.94:3000/ping",
                json={'mac': addr}
            )
            mjres = json.loads(mres.text)
            logging.debug(mjres)
            if not mjres['success']:
                logging.critical("Erro requesting ping")
                sshDisconnected = True
                return
        except:
            logging.critical("No internet connection")
            sshDisconnected = True
            return


def handler(chan, host, port):
    sock = socket.socket()
    try:
        sock.connect((host, port))
    except Exception as e:
        logging.debug("Forwarding request to %s:%d failed: %r" %
                      (host, port, e))
        return

    logging.debug(
        "Connected!  Tunnel open %r -> %r -> %r"
        % (chan.origin_addr, chan.getpeername(), (host, port))
    )
    while True:
        r, w, x = select.select([sock, chan], [], [])
        if sock in r:
            data = sock.recv(1024)
            if len(data) == 0:
                break
            chan.send(data)
        if chan in r:
            data = chan.recv(1024)
            if len(data) == 0:
                break
            sock.send(data)
    chan.close()
    sock.close()
    logging.debug("Tunnel closed from %r" % (chan.origin_addr,))


def reverse_forward_tunnel(server_port, remote_host, remote_port, transport, addr):
    transport.request_port_forward("", server_port)
    global sshDisconnected
    thrr = threading.Thread(target=checkRemoteHost,
                            args=(server_port, addr))
    thrr.start()
    while not sshDisconnected:
        chan = transport.accept(10)
        if chan is None:
            continue
        thr = threading.Thread(
            target=handler, args=(chan, remote_host, remote_port)
        )
        thr.setDaemon(True)
        thr.start()
    sshDisconnected = False
    return

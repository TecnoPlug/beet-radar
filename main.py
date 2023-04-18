from genericpath import isfile
from http.server import BaseHTTPRequestHandler, HTTPServer
from math import floor
from os import listdir
from posixpath import join
from config.init import *
from kld7 import KLD7
from socketserver import ThreadingMixIn
from kld7.device import RadarParamProxy
from pysondb import db
from collections import deque, ChainMap
from datetime import datetime
from zipfile import ZipFile
from rtunnel import enable_ap_mode, reverse_forward_tunnel

import paramiko
import threading
import numpy as np
import cv2
import multitasking
import logging
import json
import csv
import copy
import tarfile
import re
import uuid
import sys
import urllib.request

import time

multitasking.set_max_threads(10)

logging.root.setLevel(logging.NOTSET)
logging.basicConfig(level=logging.CRITICAL)

config = {}
isConfigFileUpdating = False

requireRadarReload = False
exitSystem = False
radarTarget = None

cameraCapture = True
cameraCapturing = False
cameraId = 1
cameraImgCapture = {}
cameraLiveCapture = {}
cameraOldImgCapture = {}
cameraBusy = False

mdb = db.getDb('/usr/share/radar/db.json')
car_cascade = cv2.CascadeClassifier('cars.xml')

cameraConnectionIntents = 10

processingData = False
detection = {}
detectionLastCapture = datetime.now()
detectionStartTime = datetime.now()

webClientConnected = False


def clear_detection():
    global detection
    detection = {
        'capturando': False,
        'infraccion': False,
        'velocidad': 0
    }


clear_detection()


def load_config_file(file):
    global config
    with open('config_default.json', "r") as fc:
        gconfig = json.load(fc)
        fc.close()
        if (file != 'config_default.json'):
            with open(file, "r") as f:
                config = json.load(f)
                f.close()
                if config['version'] < gconfig['version']:
                    logging.info('Update version')
                    config = ChainMap(config, gconfig)
                    # config = gconfig
        else:
            config = gconfig
        if config['contador']['fecha_inicio'] == None:
            config['contador']['fecha_inicio'] = datetime.now().isoformat()


def load_config():
    try:
        load_config_file('/usr/share/radar/config.json')
    except:
        logging.critical('Err in config file')
        load_config_file("config_default.json")
        save_config()


def save_config():
    global isConfigFileUpdating
    while isConfigFileUpdating:
        '''waiting'''
    isConfigFileUpdating = True
    with open('/usr/share/radar/config.json', "w") as f:
        f.write(json.dumps(config))
        f.close()
    isConfigFileUpdating = False


def setConfigRadar(radar):
    logging.info('Saving config radar')
    for item in config['config']['radar']:
        logging.info("set param %s >> %d", item['name'],
                     item['value'])
        radar._set_param(item['name'], item['value'])


@ multitasking.task
def startRemoteService():
    global config, webClientConnected

    if len(sys.argv) > 1:
        if sys.argv[1] == 'test':
            logging.info('Start test mode')
            return

    if not 'uuid' in config:
        config['uuid'] = (':'.join(re.findall('..', '%012x' % uuid.getnode())))
        save_config()

    addr = config['uuid']

    logging.info(f"Remote addr {addr}")

    while True:

        logging.critical("Switch to AP Mode")

        webClientConnected = True

        time.sleep(10)

        os.system("nmcli r wifi off")
        os.system("rfkill unblock all")
        os.system(
            f"create_ap wlan0 lo \"{config['config']['dispositivo']['nombre']}-{config['config']['dispositivo']['ubicacion']}\" \"GiroVision2323\"")

        while True:
            '''Infinite'''


@multitasking.task
def startRadar():
    global radarTarget, requireRadarReload, detection, config, detectionLastCapture, mdb, cameraCapture, detectionStartTime

    font = cv2.FONT_HERSHEY_SIMPLEX
    logging.info('Starting radar...\n')
    with KLD7("/dev/ttyS2") as radar:
        logging.debug('Radar listen...\n')

        members = [attr for attr in dir(radar._param_proxy) if not callable(
            getattr(radar._param_proxy, attr)) and not attr.startswith("_")]
        config['config']['radar'] = []

        for item in members:
            config['config']['radar'].append({
                'name': item,
                'desc': radar.params.__class__.__dict__[item].desc(),
                'value': radar._get_param(item)
            })

        try:
            for t in radar.stream_TDAT():
                radarTarget = t
                if not t == None:
                    logging.info(str(t))
                if config['config']['deteccion']['activo'] and not t == None and cameraCapturing:

                    direction = '-' if t.speed < 0 else '+'
                    speed = abs(
                        t.speed) * config['config']['deteccion']['factor_correccion']

                    if speed >= config['config']['deteccion']['velocidad_minima'] and (config['config']['deteccion']['sentido'] == '*' or config['config']['deteccion']['sentido'] == direction):

                        tdelta = datetime.now()-detectionLastCapture
                        if tdelta.microseconds > 200000:

                            if not detection['capturando']:
                                detectionStartTime = datetime.now()
                                logging.info('Start capture')
                                detection['fecha'] = f"{detectionStartTime.strftime('%d/%m/%Y %H:%M:%S')}",
                                detection['capturando'] = True
                                detection['cant'] = 0
                                detection['velocidad'] = 0
                                detection['max_speed'] = 0
                                detection['infraccion'] = False
                                detection['status'] = 'Trafico normal'
                                detection['direccion'] = direction
                                strSpeed = f"{floor(speed)}".rjust(3, '0')
                                detection[
                                    'filepath'] = f"/usr/share/radar/captures/{detectionStartTime.strftime('%Y/%m/%d/%H%M%S')}_{direction}{strSpeed}_2"

                            detection['velocidad'] = floor(speed)
                            if detection['max_speed'] < speed:
                                detection['max_speed'] = floor(speed)

                            if speed >= config['config']['deteccion']['velocidad']:
                                if not detection['infraccion']:
                                    logging.info('Start infraction')
                                    detection['status'] = 'Infraccion'
                                    os.makedirs(detection['filepath'])
                                    detection['infraccion'] = True

                                detection['cant'] += 1
                                strcImg = f"{detection['cant']}".rjust(3, '0')

                                # cameraCapture = False

                                # while cameraCapturing:
                                #     '''wait'''

                                logging.info('Save file')

                                w, h = 250, 100
                                x, y = cameraImgCapture.shape[1]-w, 0
                                sub_img = cameraImgCapture[y:y+h, x:x+w]
                                white_rect = np.ones(
                                    sub_img.shape, dtype=np.uint8) * 255
                                sub_img = cv2.rectangle(
                                    sub_img, (1, 1), (w-1, h-1), (0, 0, 0), 1)

                                res = cv2.addWeighted(
                                    sub_img, 0.3, white_rect, 0.7, 1.0)
                                cameraImgCapture[y:y+h, x:x+w] = res

                                cv2.putText(
                                    img=cameraImgCapture,
                                    text=config['config']['dispositivo']['ubicacion'],
                                    org=(x+5, y+20),
                                    fontFace=font,
                                    fontScale=0.5,
                                    thickness=1,
                                    color=(0, 0, 0))

                                cv2.putText(
                                    img=cameraImgCapture,
                                    text=config['config']['dispositivo']['nombre'],
                                    org=(x+5, y+37),
                                    fontFace=font,
                                    fontScale=0.5,
                                    thickness=1,
                                    color=(0, 0, 0))

                                cv2.putText(
                                    img=cameraImgCapture,
                                    text=f"Vel. Max.: {config['config']['deteccion']['velocidad']} Km/h",
                                    org=(x+5, y+54),
                                    fontFace=font,
                                    fontScale=0.5,
                                    thickness=1,
                                    color=(0, 0, 0))

                                cv2.putText(
                                    img=cameraImgCapture,
                                    text=f"Vel. Detec.: {floor(speed)} Km/h",
                                    org=(x+5, y+71),
                                    fontFace=font,
                                    fontScale=0.5,
                                    thickness=1,
                                    color=(0, 0, 0))

                                cv2.putText(
                                    img=cameraImgCapture,
                                    text=f"{datetime.now().strftime('%d/%m/%Y %H:%M:%S')}",
                                    org=(x+5, y+88),
                                    fontFace=font,
                                    fontScale=0.5,
                                    thickness=1,
                                    color=(0, 0, 0))

                                try:
                                    cv2.imwrite(
                                        f"{detection['filepath']}/{config['config']['dispositivo']['ubicacion']}-{config['config']['dispositivo']['nombre']}-{datetime.now().strftime('%d-%m-%Y-%H-%M-%S')}-{config['config']['deteccion']['velocidad']}-{floor(speed)}-{strcImg}.jpg", cameraImgCapture)
                                except:
                                    logging.critical('Image insert err')

                                # cameraCapture = True
                            logging.info('End capture')
                            detectionLastCapture = datetime.now()
                        logging.info('CHECK DELTA END')

                tdelta = datetime.now()-detectionLastCapture

                if tdelta.seconds > 1 and detection['capturando']:
                    '''Finalizo el proceso de captura'''
                    logging.info('End all capture')
                    if detection['infraccion']:
                        logging.info('Save infraccion')
                        print(detection)
                        mdb.add(detection)
                    fdir = f"/usr/share/radar/captures/{datetime.now().strftime('%Y/%m/%d')}"
                    if not os.path.exists(fdir):
                        os.makedirs(fdir)
                    with open(f"{fdir}/contador_{datetime.now().strftime('%Y-%m-%d')}.csv", 'a') as f:
                        writer = csv.writer(f)
                        writer.writerow(
                            [f"{config['config']['dispositivo']['nombre']}|{datetime.now().strftime('%Y-%m-%d|%H:%M:%S')}|{config['config']['deteccion']['velocidad']}|{detection['velocidad']}|{detection['status']}"])
                    clear_detection()

                if (requireRadarReload):
                    requireRadarReload = False
                    setConfigRadar(radar)
        except Exception as err:
            logging.critical(f"Unexpected {err=}, {type(err)=}")
            '''node'''


class ServerHandler(BaseHTTPRequestHandler):

    def do_OPTIONS(self):
        self.send_response(200, "ok")
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header("Access-Control-Allow-Headers", "X-Requested-With")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        self.respond()

    def do_POST(self):
        self.respond()

    def handle_http(self):
        global config, requireRadarReload, mdb, webClientConnected, cameraCapture

        webClientConnected = True
        status = 200
        content_type = "application/json"
        res = {
            'success': False,
            'sent': 'OK'
        }

        if (self.path == '/getConfig'):
            res['data'] = config['config']
            res['success'] = True

        elif (self.path == '/getResume'):
            logging.info('Get resume')
            res['data'] = config['contador']
            res['success'] = True

        elif (self.path == '/getHistory'):
            res['data'] = mdb.getAll()
            res['success'] = True

        elif (self.path == '/resetDevice'):
            os.system('reboot')

        elif (self.path == '/listFiles'):
            res['success'] = True
            res['list'] = []
            length = int(self.headers.get('content-length'))
            try:
                mpath = json.loads(self.rfile.read(length))
                res['list'] = [f for f in listdir(
                    mpath['path']) if isfile(join(mpath['path'], f))]

            except NameError:
                logging.error(NameError)

        elif '/captures' in self.path:
            fileName = '/usr/share/radar' + urllib.parse.unquote(self.path)
            f = open(fileName, 'rb')
            fs = os.fstat(f.fileno())
            self.send_response(200)
            self.send_header('Content-type', 'image/jpeg')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Headers',
                             'Origin, X-Requested-With, Content-Type, Accept')
            self.send_header("Content-Length", str(fs.st_size))
            self.end_headers()
            self.wfile.write(f.read())
            f.close()

            return None

        elif (self.path == '/clearData'):

            os.remove('/usr/share/radar/db.json')
            mdb = db.getDb('/usr/share/radar/db.json')
            config['contador']['infracciones'] = 0
            config['contador']['detectados'] = 0
            config['contador']['minutos_activos'] = 0
            save_config()
            os.system('rm -R /usr/share/radar/captures/*')
            os.system('rm -R /usr/share/radar/tars/*')
            os.system('rm /usr/share/radar/download.zip')

            res['success'] = True

        elif (self.path == '/downloadData'):
            cameraCapture = False

            while cameraCapturing:
                '''Waiting release camera'''

            logging.info('Creating tar file...')

            if not os.path.exists('/usr/share/radar/tars'):
                os.makedirs('/usr/share/radar/tars')

            with tarfile.open(
                    f"/usr/share/radar/tars/{config['config']['dispositivo']['ubicacion']}-{config['config']['dispositivo']['nombre']}-{datetime.now().strftime('%Y-%m-%d')}.tar", "a") as archive:

                fdir = f"/usr/share/radar/captures/{datetime.now().strftime('%Y/%m/%d')}"
                archive.add(
                    fdir,  arcname=f"var/www/html/fotos/{datetime.now().strftime('%Y/%m/%d')}")

                with open('config/fr.conf', 'r') as file:
                    filedata = file.read()

                filedata = filedata.replace(
                    '/UBICACION/', config['config']['dispositivo']['ubicacion'])
                filedata = filedata.replace(
                    '/NOMBRE/', config['config']['dispositivo']['nombre'])
                filedata = filedata.replace(
                    '/FECHA/', datetime.now().strftime('%Y-%m-%d'))
                filedata = filedata.replace(
                    '/HORA/', datetime.now().strftime('%H:%M'))

                with open('/usr/share/radar/captures/fr.conf', 'w') as file:
                    file.write(filedata)

                archive.add('/usr/share/radar/captures/fr.conf',
                            arcname=f"var/www/html/fr.conf")

                archive.close()

                logging.info('Saving data...')

            with ZipFile('/usr/share/radar/download.zip', 'w') as zip_object:
                for file in os.listdir('/usr/share/radar/tars'):
                    if os.path.isfile(os.path.join('/usr/share/radar/tars', file)):
                        zip_object.write(os.path.join(
                            '/usr/share/radar/tars', file), file)
            f = open('/usr/share/radar/download.zip', 'rb')
            fs = os.fstat(f.fileno())
            self.send_response(200)
            self.send_header('Content-type', 'application/zip')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header("Content-Length", str(fs.st_size))
            self.send_header('Access-Control-Allow-Headers',
                             'Origin, X-Requested-With, Content-Type, Accept')
            self.end_headers()
            self.wfile.write(f.read())
            f.close()
            cameraCapture = True

            return None

        elif (self.path == '/getLiveData'):
            res['success'] = True
            res['data'] = {
                'detection': False,
                'camera': cameraCapturing
            }
            if (radarTarget != None):
                res['data']['detection'] = True
                res['data']['speed'] = radarTarget.speed
                res['data']['distance'] = radarTarget.distance
                res['data']['angle'] = radarTarget.angle

        elif (self.path == '/setConfig'):
            res['success'] = True
            length = int(self.headers.get('content-length'))
            try:
                config['config'] = json.loads(self.rfile.read(length))
                save_config()
            except NameError:
                logging.error(NameError)
            requireRadarReload = True

        elif ('/shot.jpg' in self.path):
            self.send_response(200)
            self.send_header('Content-type', 'image/jpeg')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Headers',
                             'Origin, X-Requested-With, Content-Type, Accept')
            self.end_headers()
            while cameraBusy:
                time.sleep(1/30)
                '''No picture'''
            try:
                img_str = cv2.imencode('.jpg', cameraLiveCapture)[
                    1].tostring()
                self.wfile.write(img_str)
            except:
                logging.debug('Img err')
            return None

        elif (self.path == '/stream.mjpg'):
            logging.info('Image stream enable')
            self.send_response(status)
            self.send_header(
                'Content-type', 'multipart/x-mixed-replace; boundary=--jpgboundary')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Headers',
                             'Origin, X-Requested-With, Content-Type, Accept')
            self.end_headers()

            while True:
                try:
                    while cameraBusy:
                        '''Camera busy'''
                    img = cameraLiveCapture
                    if config['config']['deteccion']['area']['y1'] > 0 or config['config']['deteccion']['area']['y2'] > 0:
                        img = img[config['config']['deteccion']['area']['y1']:config['config']['deteccion']['area']['y2'],
                                  config['config']['deteccion']['area']['x1']: config['config']['deteccion']['area']['x2']]
                    img_str = cv2.imencode('.jpg', img)[
                        1].tostring()
                    self.send_header('Content-type', 'image/jpeg')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.send_header('Access-Control-Allow-Headers',
                                     'Origin, X-Requested-With, Content-Type, Accept')
                    self.end_headers()
                    self.wfile.write(img_str)
                    self.wfile.write(b"\r\n--jpgboundary\r\n")
                except Exception as err:
                    logging.critical(f"Unexpected {err=}, {type(err)=}")
                    break
            logging.info('Image stream disable')
            return None

        self.send_response(status)
        self.send_header('Content-type', content_type)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Headers',
                         'Origin, X-Requested-With, Content-Type, Accept')
        self.end_headers()
        return bytes(json.dumps(res), "UTF-8")

    def respond(self):
        content = self.handle_http()
        if content != None:
            self.wfile.write(content)

    def log_message(self, format, *args):
        return


@multitasking.task
def startCamera():
    global cameraId, cameraBusy, cameraConnectionIntents, config, cameraCapturing, cameraLiveCapture, cameraImgCapture

    while True:
        if cameraCapture:
            try:
                logging.info('Try connect camera')
                cap = cv2.VideoCapture(
                    f"v4l2src device=/dev/video{cameraId} ! image/jpeg,framerate=30/1,width=1920,height=1080 \
                    ! jpegdec ! videoconvert ! appsink max-buffers=1 drop=True", cv2.CAP_GSTREAMER)
                if cap.isOpened():
                    print(f'Camera index available: {cameraId}')
                    while (True):
                        if cameraCapture:
                            try:
                                if not cameraCapturing:
                                    logging.info('Enable capturing')
                                    cameraCapturing = True

                                cameraBusy = True
                                ret, cameraLiveCapture = cap.read()
                                if cameraLiveCapture.any():
                                    if (config['config']['deteccion']['auto']):
                                        if cameraOldImgCapture == None:
                                            cameraOldImgCapture = cameraLiveCapture.copy()
                                        else:
                                            diff = cv2.absdiff(
                                                cameraOldImgCapture, cameraLiveCapture)
                                            cameraOldImgCapture = cameraLiveCapture.copy()
                                            diff_gray = cv2.cvtColor(
                                                diff, cv2.COLOR_BGR2GRAY)
                                            diff_blur = cv2.GaussianBlur(
                                                diff_gray, (5, 5), 0)
                                            _, thresh_bin = cv2.threshold(
                                                diff_blur, 20, 255, cv2.THRESH_BINARY)
                                            contours, _ = cv2.findContours(
                                                thresh_bin, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                                            for contour in contours:
                                                x, y, w, h = cv2.boundingRect(
                                                    contour)
                                                if w >= 300 and h >= 300:
                                                    cameraImgCapture = cameraLiveCapture[y:y+h, x:x+w]
                                    else:
                                        if config['config']['deteccion']['area']['y1'] > 0 or config['config']['deteccion']['area']['y2'] > 0:
                                            cameraImgCapture = cameraLiveCapture[config['config']['deteccion']['area']['y1']:config['config']['deteccion']
                                                                                 ['area']['y2'], config['config']['deteccion']['area']['x1']: config['config']['deteccion']['area']['x2']]
                                cameraBusy = False

                            except Exception as err:
                                logging.critical(
                                    f"Unexpected {err=}, {str(err)=}")
                                cap.release()
                                startCamera()
                                return
                        else:
                            logging.info('End camera capture')
                            if cameraCapturing:
                                cameraCapturing = False
                            cap.release()
                            break
                        time.sleep(1/30)
                else:
                    logging.info('ERROR!!!!! Camera not found')
                    time.sleep(cameraConnectionIntents)
                    if cameraConnectionIntents < 60:
                        cameraConnectionIntents += 1
                    cameraId = cameraId+1
                    if cameraId == 5:
                        cameraId = 1
            except:
                # logging.critical('ERROR!!!!! Camera not found')
                time.sleep(cameraConnectionIntents)
                if cameraConnectionIntents < 60:
                    cameraConnectionIntents += 1
                cameraId = cameraId+1
                if cameraId == 5:
                    cameraId = 1


def startCronJobs():
    config['contador']['minutos_activos'] += 1
    save_config()


ThreadingMixIn.daemon_threads = True


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""


@ multitasking.task
def startWebServer():

    httpd = ThreadedHTTPServer(('', 9000), ServerHandler)
    logging.debug('Starting httpd...\n')
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    httpd.server_close()
    logging.debug('Stopping httpd...\n')


load_config()
initSystem()
startRadar()
startCamera()
startWebServer()
startRemoteService()

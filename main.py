from genericpath import isfile
from http.server import BaseHTTPRequestHandler, HTTPServer
from math import floor
from os import listdir
from posixpath import join
from config.init import *
from camera import __gstreamer_pipeline
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
import requests
import urllib.request

import time

multitasking.set_max_threads(10)

logging.root.setLevel(logging.NOTSET)
logging.basicConfig(level=logging.CRITICAL)

config = {}
isConfigFileUpdating = False

requireRadarReload = False
exitSystem = False
radarTarger = None


cameraRc = False
cameraDetected = False
cameraCapture = True
cameraId = 1
cameraLastCapture = datetime.now()

mdb = db.getDb('/usr/share/radar/db.json')
car_cascade = cv2.CascadeClassifier('cars.xml')

cameraFrameList = deque([], 20)
cameraConnectionIntents = 10

processList = deque([], 3)
processListUpdating = False

webClientConnected = False


class Detection():

    direction = '+'
    max_speed = 0
    min_speed = 0
    start_time = datetime.now()
    last_time = datetime.now()
    captures = deque([], 20)

    def close(self):
        global cameraCapture, processListUpdating
        cameraCapture = False
        self.captures = copy.deepcopy(cameraFrameList)
        cameraCapture = True

    def save_data(self):
        global mdb
        font = cv2.FONT_HERSHEY_SIMPLEX
        timeCalc = datetime.now()
        logging.info(f"Start processing data at {timeCalc}")
        opt = config['config']

        config['contador']['detectados'] += 1
        config['contador']['total_detectados'] += 1

        d = {
            'velocidad_deteccion': opt['deteccion']['velocidad_minima'],
            'velocidad_infraccion': opt['deteccion']['velocidad'],
            'velocidad': floor(self.max_speed),
            'velocidad_minima': floor(self.min_speed),
            'direccion': self.direction,
            'fecha': self.start_time.strftime('%d/%m/%Y %H:%M:%S'),
            'dispositivo': opt['dispositivo'],
            'infraccion': False,
            'status': 'Trafico normal',
            'filepath': ''
        }

        if (self.max_speed > d['velocidad_infraccion']):
            ''''Infraccion'''
            logging.critical('Infraction detect')
            d['infraccion'] = True
            d['status'] = 'Infraccion'
            config['contador']['infracciones'] += 1
            config['contador']['total_infracciones'] += 1

            strSpeed = f"{d['velocidad']}".rjust(3, '0')
            directory = f"/usr/share/radar/captures/{self.start_time.strftime('%Y/%m/%d/%H%M%S')}_{d['direccion']}{strSpeed}_2"
            d['filepath'] = directory

            os.makedirs(directory)

            cImg = 0
            oImg = np.zeros((100, 100, 3), dtype=np.uint8)
            imgPass = False

            for img in self.captures:

                if (opt['deteccion']['auto']):
                    '''Buscar autos en la imagen'''
                    if oImg.any():
                        diff = cv2.absdiff(oImg, img)
                        oImg = img.copy()
                        diff_gray = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
                        diff_blur = cv2.GaussianBlur(diff_gray, (5, 5), 0)
                        _, thresh_bin = cv2.threshold(
                            diff_blur, 20, 255, cv2.THRESH_BINARY)
                        contours, _ = cv2.findContours(
                            thresh_bin, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                        for contour in contours:
                            x, y, w, h = cv2.boundingRect(contour)
                            if w >= 300 and h >= 300:
                                img = img[y:y+h, x:x+w]
                                imgPass = True
                                break
                else:
                    if opt['deteccion']['area']['y1'] > 0 or opt['deteccion']['area']['y2'] > 0:
                        img = img[opt['deteccion']['area']['y1']:opt['deteccion']['area']['y2'],
                                  opt['deteccion']['area']['x1']: opt['deteccion']['area']['x2']]
                    imgPass = True

                if imgPass:

                    cImg += 1
                    strcImg = f"{cImg}".rjust(3, '0')
                    w, h = 250, 100
                    x, y = img.shape[1]-w, 0
                    sub_img = img[y:y+h, x:x+w]
                    white_rect = np.ones(sub_img.shape, dtype=np.uint8) * 255
                    sub_img = cv2.rectangle(
                        sub_img, (1, 1), (w-1, h-1), (0, 0, 0), 1)

                    res = cv2.addWeighted(sub_img, 0.3, white_rect, 0.7, 1.0)
                    img[y:y+h, x:x+w] = res

                    cv2.putText(
                        img=img,
                        text=d['dispositivo']['ubicacion'],
                        org=(x+5, y+20),
                        fontFace=font,
                        fontScale=0.5,
                        thickness=1,
                        color=(0, 0, 0))

                    cv2.putText(
                        img=img,
                        text=d['dispositivo']['nombre'],
                        org=(x+5, y+37),
                        fontFace=font,
                        fontScale=0.5,
                        thickness=1,
                        color=(0, 0, 0))

                    cv2.putText(
                        img=img,
                        text=f"Vel. Max.: {d['velocidad_infraccion']} Km/h",
                        org=(x+5, y+54),
                        fontFace=font,
                        fontScale=0.5,
                        thickness=1,
                        color=(0, 0, 0))

                    cv2.putText(
                        img=img,
                        text=f"Vel. Detec.: {d['velocidad_deteccion']} Km/h",
                        org=(x+5, y+71),
                        fontFace=font,
                        fontScale=0.5,
                        thickness=1,
                        color=(0, 0, 0))

                    cv2.putText(
                        img=img,
                        text=f"{self.start_time.strftime('%d/%m/%Y %H:%M:%S')}",
                        org=(x+5, y+88),
                        fontFace=font,
                        fontScale=0.5,
                        thickness=1,
                        color=(0, 0, 0))

                    try:
                        cv2.imwrite(
                            f"{directory}/{d['dispositivo']['ubicacion']}-{d['dispositivo']['nombre']}-{self.start_time.strftime('%d-%m-%Y-%H-%M-%S')}-{d['velocidad_infraccion']}-{d['velocidad']}-{strcImg}.jpg", img)
                    except:
                        logging.critical('Image insert err')

        fdir = f"/usr/share/radar/captures/{self.start_time.strftime('%Y/%m/%d')}"
        if not os.path.exists(fdir):
            os.makedirs(fdir)

        with open(f"{fdir}/contador_{self.start_time.strftime('%Y-%m-%d')}.csv", 'a') as f:
            writer = csv.writer(f)
            writer.writerow(
                [f"{d['dispositivo']['nombre']}|{self.start_time.strftime('%Y-%m-%d|%H:%M:%S')}|{d['velocidad']}|{d['velocidad_infraccion']}|{d['status']}"])

        logging.info('Creating tar file...')

        if not os.path.exists('/usr/share/radar/tars'):
            os.makedirs('/usr/share/radar/tars')

        with tarfile.open(
                f"/usr/share/radar/tars/{d['dispositivo']['ubicacion']}-{d['dispositivo']['nombre']}-{self.start_time.strftime('%Y-%m-%d')}.tar", "a") as archive:

            archive.add(
                fdir,  arcname=f"var/www/html/fotos/{self.start_time.strftime('%Y/%m/%d')}")

            with open('config/fr.conf', 'r') as file:
                filedata = file.read()

            filedata = filedata.replace(
                '/UBICACION/', d['dispositivo']['ubicacion'])
            filedata = filedata.replace('/NOMBRE/', d['dispositivo']['nombre'])
            filedata = filedata.replace(
                '/FECHA/', self.start_time.strftime('%Y-%m-%d'))
            filedata = filedata.replace(
                '/HORA/', self.start_time.strftime('%H:%M'))

            with open('/usr/share/radar/captures/fr.conf', 'w') as file:
                file.write(filedata)

            archive.add('/usr/share/radar/captures/fr.conf',
                        arcname=f"var/www/html/fr.conf")

            archive.close()

        logging.info('Saving data...')

        if d['infraccion']:
            mdb.add(d)

        try:
            save_config()
        except:

            '''erro'''
        tdelta = datetime.now()-timeCalc
        logging.info(
            f"Process data in {tdelta.microseconds / 100000} seconds")


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
        global config, requireRadarReload, mdb, webClientConnected

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

            return None

        elif (self.path == '/getLiveData'):
            res['success'] = True
            res['data'] = {
                'detection': False,
                'camera': cameraDetected
            }
            if (radarTarger != None):
                res['data']['detection'] = True
                res['data']['speed'] = radarTarger.speed
                res['data']['distance'] = radarTarger.distance
                res['data']['angle'] = radarTarger.angle

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
            while not cameraRc:
                '''No picture'''
            try:
                img_str = cv2.imencode('.jpg', cameraLastFrame)[
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
                    if not cameraRc:
                        continue
                    img = cameraLastFrame
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
                    time.sleep(1/30)
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


def withInternet():
    try:
        urllib.request.urlopen('http://168.197.51.94')
        return True
    except:
        return False


def setConfigRadar(radar):
    logging.info('Saving config radar')
    for item in config['config']['radar']:
        logging.info("set param %s >> %d", item['name'],
                     item['value'])
        radar._set_param(item['name'], item['value'])


@ multitasking.task
def startRemoteService():
    global config, webClientConnected

    if not 'uuid' in config:
        config['uuid'] = (':'.join(re.findall('..', '%012x' % uuid.getnode())))
        save_config()

    addr = config['uuid']

    logging.info(f"Remote addr {addr}")

    while True:
        # try:
        #     res = requests.post(
        #         'http://168.197.51.94:3000/check-port', json={
        #             'mac': addr,
        #             'name': config['config']['dispositivo']['nombre'],
        #             'location': config['config']['dispositivo']['ubicacion'],
        #             'type': 'radar'
        #         })
        #     jres = json.loads(res.text)
        #     logging.info(jres)
        #     if jres['success']:

        #         logging.info('Start tunneling...')
        #         ssh = paramiko.SSHClient()
        #         ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        #         ssh.connect(username="radar", hostname="168.197.51.94",
        #                     port=2257, password="Yhbg5tTfgRfdsGvFgfrPolWo")
        #         transport = ssh.get_transport()
        #         reverse_forward_tunnel(
        #             5000 + jres['port'], "localhost", 9000, transport, addr)
        #         logging.info('Server end')
        #         if ssh:
        #             ssh.close()
        # except Exception as e:
        #     logging.critical(
        #         "*** Failed to connect o server")

        logging.critical("Switch to AP Mode")

        webClientConnected = True

        time.sleep(10)

        os.system("nmcli r wifi off")
        os.system("rfkill unblock all")
        os.system(
            f"create_ap wlan0 lo \"{config['config']['dispositivo']['nombre']}-{config['config']['dispositivo']['ubicacion']}\" \"GiroVision2323\"")
        # enable_ap_mode(config)

        while True:
            '''Infinite'''

        # SI EN 30 MINUTOS NO HAY CONEXION SALGO DE MODO AP
        while webClientConnected:
            webClientConnected = False
            time.sleep(30 * 60)

        os.system("turn-wifi-into-apmode no")
        logging.critical("Exit AP Mode")

        time.sleep(30)


@multitasking.task
def startRadar():
    global radarTarger, requireRadarReload, processList, processListUpdating, config

    logging.info('Starting radar...\n')
    with KLD7("/dev/ttyS2") as radar:
        logging.debug('Radar listen...\n')

        detectionRe = None
        detectionGo = None

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

                radarTarger = t
                # logging.info(t)
                if not t == None:
                    opt = config['config']['deteccion']
                    logging.debug(t)
                    if opt['activo']:

                        # check the capture
                        direction = '-' if t.speed < 0 else '+'
                        speed = abs(t.speed) * opt['factor_correccion']
                        if speed >= opt['velocidad_minima'] and (opt['sentido'] == '*' or opt['sentido'] == direction):
                            if direction == '+':
                                if detectionRe == None:
                                    detectionRe = Detection()
                                    detectionRe.direction = '+'
                                    detectionRe.start_time = datetime.now()
                                    detectionRe.min_speed = speed
                                detection = detectionRe
                            else:
                                if detectionGo == None:
                                    detectionGo = Detection()
                                    detectionGo.direction = '-'
                                    detectionGo.start_time = datetime.now()
                                    detectionGo.min_speed = speed
                                detection = detectionGo
                            detection.last_time = datetime.now()
                            if detection.max_speed < speed:
                                detection.max_speed = speed
                            if detection.min_speed > speed:
                                detection.min_speed = speed

                if cameraCapture:
                    if detectionRe != None:
                        tdelta = datetime.now() - detectionRe.last_time
                        if (tdelta.microseconds > 20000):
                            logging.info('Detection Re end')
                            detectionRe.close()
                            while processListUpdating:
                                '''wait'''
                            processListUpdating = True
                            processList.append(copy.deepcopy(detectionRe))
                            processListUpdating = False
                            detectionRe = None

                    if detectionGo != None:
                        tdelta = datetime.now() - detectionGo.last_time
                        if (tdelta.microseconds > 20000):
                            logging.info('Detection Go end')
                            detectionGo.close()
                            while processListUpdating:
                                '''wait'''
                            processListUpdating = True
                            processList.append(copy.deepcopy(detectionGo))
                            processListUpdating = False
                            detectionGo = None

                if (requireRadarReload):
                    requireRadarReload = False
                    setConfigRadar(radar)
        except:
            '''node'''


@multitasking.task
def processListTask():
    global processListUpdating, processList
    while True:
        try:
            while processList:
                while processListUpdating:
                    '''Updating'''
                processListUpdating = True
                detect = processList.pop()
                processListUpdating = False
                detect.save_data()
        except NameError:
            logging.critical(NameError)
            '''Error in process data'''
        time.sleep(5)


@multitasking.task
def startCamera():
    global cameraFrameList, cameraRc, cameraId, cameraDetected, cameraLastCapture, cameraConnectionIntents, cameraLastFrame, config
    try:
        # cap = cv2.VideoCapture(cameraId)
        # cap = cv2.VideoCapture(
        #    f"v4l2src device=/dev/video{cameraId} ! image/jpeg,framerate=30/1,width=640,height=480,type=capture \
        #    ,force-aspect-ratio=false ! jpegdec ! videoconvert ! video/x-raw ! appsink", cv2.CAP_GSTREAMER)
        cap = cv2.VideoCapture(
            f"v4l2src device=/dev/video{cameraId} ! image/jpeg,framerate=30/1,width=1920,height=1080 \
            ! jpegdec ! videoconvert ! appsink max-buffers=1 drop=True", cv2.CAP_GSTREAMER)
        # cap = cv2.VideoCapture(
        #     f"v4l2src device=/dev/video{cameraId} ! queue ! video/x-h264,width=1920,height=1080,framerate=30/1 ! avdec_h264 ! videoconvert \
        #     ! appsink", cv2.CAP_GSTREAMER)
        # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        # cap.set(cv2.CAP_PROP_FPS, 30)
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        # cap = cv2.VideoCapture(cameraId)
        # # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('P', 'I', 'M', '1'))
        # cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        # out = cv2.VideoWriter('output.mkv', fourcc, 30.0, (1920, 1080))
        if cap.isOpened():
            print(f'Camera index available: {cameraId}')
            cameraDetected = True
            while (True):
                if cameraCapture:
                    try:
                        ret, cameraLastFrame = cap.read()
                        cameraRc = ret
                        # print(frame.shape[1::-1])
                    except:
                        cap.release()
                        startCamera()
                        return
                    tdelta = datetime.now() - cameraLastCapture
                    if ret and cameraCapture and tdelta.microseconds > 200000:
                        imgPass = False
                        if (config['config']['deteccion']['auto']):
                            '''Buscar autos en la imagen'''
                            if oImg.any():
                                diff = cv2.absdiff(oImg, cameraLastFrame)
                                oImg = cameraLastFrame.copy()
                                diff_gray = cv2.cvtColor(
                                    diff, cv2.COLOR_BGR2GRAY)
                                diff_blur = cv2.GaussianBlur(
                                    diff_gray, (5, 5), 0)
                                _, thresh_bin = cv2.threshold(
                                    diff_blur, 20, 255, cv2.THRESH_BINARY)
                                contours, _ = cv2.findContours(
                                    thresh_bin, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                                for contour in contours:
                                    x, y, w, h = cv2.boundingRect(contour)
                                    if w >= 300 and h >= 300:
                                        frame = cameraLastFrame[y:y+h, x:x+w]
                                        imgPass = True
                                        break
                        else:
                            if config['config']['deteccion']['area']['y1'] > 0 or config['config']['deteccion']['area']['y2'] > 0:
                                frame = cameraLastFrame[config['config']['deteccion']['area']['y1']:config['config']['deteccion']['area']['y2'],
                                                        config['config']['deteccion']['area']['x1']: config['config']['deteccion']['area']['x2']]
                            imgPass = True
                        if imgPass:
                            cameraLastCapture = datetime.now()
                            cameraFrameList.append(frame)
                    if (exitSystem):
                        break
                time.sleep(1/30)
            cap.release()
    except Exception as e:
        print(e)
        '''Error in camera'''

    logging.critical('Camera not found')
    time.sleep(cameraConnectionIntents)
    if cameraConnectionIntents < 60:
        cameraConnectionIntents += 1
    cameraId = cameraId+1
    if cameraId == 5:
        cameraId = 1
    startCamera()


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


def startCronJobs():
    config['contador']['minutos_activos'] += 1
    save_config()


load_config()
initSystem()

threading.Timer(60, startCronJobs).start()

startRadar()
startWebServer()
startCamera()
# startRemoteService()
processListTask()

# app_serial_gui.py
import sys, time, collections, math
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
import pyqtgraph as pg
from pyqtgraph.opengl import GLViewWidget, GLLinePlotItem, GLGridItem
from PyQt5.QtGui import QVector3D
from PyQt5.QtCore import Qt
import numpy as np

# =================== PROTOCOLO ===================
HEADER_ADC  = 0x02      # ADC + DAC
HEADER_MPU  = 0x04      # Solo IMU
HEADER_FULL = 0x07      # Full: ADC + DAC + IMU

ACK  = bytes([0x04])
NACK = bytes([0x07])

PROTO = {
    HEADER_FULL: {
        "name": "Full",
        "frame_len": 24,  # 1 hdr + 8 adc + 1 dac + 12 imu + 2 crc
        "have_adc": True, "have_dac": True, "have_imu": True,
        "OFF": {
            # En main.c: se envía LSB,MSB
            "ADC0": 1, "ADC1": 3, "ADC2": 5, "ADC3": 7,
            "DAC":  9,
            "AX":  10, "AY": 12, "AZ": 14, "GX": 16, "GY": 18, "GZ": 20,
            "CRC": 22
        }
    },
    HEADER_MPU: {
        "name": "MPU 6050",
        "frame_len": 15,  # 1 hdr + 12 imu + 2 crc
        "have_adc": False, "have_dac": False, "have_imu": True,
        "OFF": {
            "AX": 1, "AY": 3, "AZ": 5, "GX": 7, "GY": 9, "GZ": 11,
            "CRC": 13
        }
    },
    HEADER_ADC: {
        "name": "Canales Analógicos",
        "frame_len": 12,  # 1 hdr + 8 adc + 1 dac + 2 crc
        "have_adc": True, "have_dac": True, "have_imu": False,
        "OFF": {
            "ADC0": 1, "ADC1": 3, "ADC2": 5, "ADC3": 7,
            "DAC":  9,
            "CRC": 10
        }
    }
}
# =================================================

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF

# Escalas MPU6050
ACCEL_SCALE = 16384.0
GYRO_SCALE  = 131.0
GRAVITY     = 9.8

# Complementario
DT    = 0.01
ALPHA = 0.7

# Escalas visuales
MAX_ACC = 15.0  # m/s²
MAX_GYR = 10.0  # °/s

# Ventana de muestras visible en XY
WIN_POINTS = 600

# ------------ Utilidades UI ------------
def bold_title(text):
    lab = QtWidgets.QLabel(text)
    lab.setAlignment(Qt.AlignHCenter)
    lab.setStyleSheet("font-weight: bold; font-size: 16px;")
    return lab

def legend_label(text):
    lab = QtWidgets.QLabel(text)
    lab.setAlignment(Qt.AlignHCenter)
    lab.setStyleSheet("font-weight: bold;")
    return lab

def value_label(prefix="", center=False):
    lab = QtWidgets.QLabel(prefix + " —")
    lab.setAlignment((Qt.AlignHCenter if center else Qt.AlignLeft) | Qt.AlignVCenter)
    lab.setStyleSheet("font-style: italic;")
    return lab

def make_frame_with_title(title:str, inner_widget:QtWidgets.QWidget) -> QtWidgets.QWidget:
    outer = QtWidgets.QWidget()
    v = QtWidgets.QVBoxLayout(outer); v.setContentsMargins(2,2,2,2); v.setSpacing(6)
    v.addWidget(bold_title(title))
    frame = QtWidgets.QFrame(); frame.setFrameShape(QtWidgets.QFrame.Box); frame.setLineWidth(1)
    fv = QtWidgets.QVBoxLayout(frame); fv.setContentsMargins(8,8,8,8); fv.setSpacing(6)
    fv.addWidget(inner_widget)
    v.addWidget(frame)
    return outer

# ------------ ViewBox con modo “seguir” ------------
class FollowViewBox(pg.ViewBox):
    """Permite zoom/pan en X. Si el usuario interactúa, desactiva 'seguir'.
       Cuando 'seguir' está activo, el gráfico usa ventana deslizante."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, enableMenu=True, **kwargs)
        self.follow = True
        self.setMouseEnabled(x=True, y=False)

    def wheelEvent(self, ev):
        self.follow = False
        super().wheelEvent(ev)

    def mouseDragEvent(self, ev, axis=None):
        if ev.button() == Qt.LeftButton:
            self.follow = False
        super().mouseDragEvent(ev, axis=axis)

# ------------ Flecha 3D ------------
class Arrow3D(GLLinePlotItem):
    def __init__(self, color=(1,1,1,1)):
        super().__init__(mode='lines', antialias=True)
        self.color = color
        self.update_vec(0,0,0)
    def update_vec(self, x, y, z):
        tip = np.array([float(x), float(y), float(z)], dtype=float)
        L = max(1e-6, float((tip**2).sum()**0.5))
        s = 0.06 * L
        a = np.array([1.0,0.0,0.0])
        if L > 0 and abs((a*tip).sum()/L) > 0.9:
            a = np.array([0.0,1.0,0.0])
        b = np.cross(tip, a)
        if (b**2).sum() > 0: b = b/((b**2).sum()**0.5)
        a = np.cross(b, tip)
        if (a**2).sum() > 0: a = a/((a**2).sum()**0.5)
        base = np.zeros(3); neck = tip - 0.1*tip
        p1 = neck + a*s; p2 = neck - a*s; p3 = neck + b*s; p4 = neck - b*s
        pts = np.array([ base, tip, tip, p1, tip, p2, tip, p3, tip, p4 ], dtype=float)
        self.setData(pos=pts, color=self.color, width=2.0)

# ------------ RPY -> Vector unitario para Roll, Pitch, Yaw ------------
def rpy_to_unit_vector(roll_deg: float, pitch_deg: float, yaw_deg: float):
    r, p, y = map(math.radians, (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    
    vx = cy*sp*cr + sy*sr
    vy = sy*sp*cr - cy*sr
    vz = cp*cr
    n = math.sqrt(vx*vx + vy*vy + vz*vz)
    return (vx/n, vy/n, vz/n) if n else (0.0, 0.0, 1.0)

# ------------ Ventana principal ------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TD3 TP3 : Monitor inercial con ESP32")
        cw = QtWidgets.QWidget(); self.setCentralWidget(cw)

        # ---------- Barra superior ----------
        self.portBox = QtWidgets.QComboBox()
        self.baudBox = QtWidgets.QComboBox()
        self.btnRefresh = QtWidgets.QToolButton(text="↻")
        self.btnConn = QtWidgets.QPushButton("Conectar")
        self.modeBox = QtWidgets.QLineEdit(); self.modeBox.setReadOnly(True); self.modeBox.setPlaceholderText("Modo: —")

        for w in (self.portBox, self.baudBox, self.modeBox, self.btnConn, self.btnRefresh):
            w.setMinimumHeight(28)
            w.setStyleSheet("font-size: 11pt;")

        labPuerto = QtWidgets.QLabel("Puerto"); labPuerto.setStyleSheet("font-size: 11pt; font-weight: bold;")
        labBaud   = QtWidgets.QLabel("Baud Rate"); labBaud.setStyleSheet("font-size: 11pt; font-weight: bold;")
        labModo   = QtWidgets.QLabel("Modo"); labModo.setStyleSheet("font-size: 11pt; font-weight: bold;")

        self.btnRefresh.clicked.connect(self.refresh_ports)
        self.btnConn.clicked.connect(self.toggle_port)
        self.refresh_ports()
        self.baudBox.addItems(["9600","19200","38400","57600","115200","230400","460800","921600"])

        # Botón para reactivar seguimiento en eje X
        self.btnFollow = QtWidgets.QPushButton("Seguir X"); self.btnFollow.setCheckable(True); self.btnFollow.setChecked(True)
        self.btnFollow.setToolTip("Cuando está activo, el gráfico avanza automáticamente.")

        top = QtWidgets.QHBoxLayout()
        top.addWidget(labPuerto); top.addWidget(self.portBox, 1); top.addWidget(self.btnRefresh)
        top.addSpacing(12); top.addWidget(labBaud); top.addWidget(self.baudBox); top.addWidget(self.btnConn)
        top.addStretch(1); top.addWidget(labModo); top.addWidget(self.modeBox); top.addSpacing(12); top.addWidget(self.btnFollow)

        # ---------- Plots ADC/DAC ----------
        pg.setConfigOptions(antialias=True)

        # ADC
        self.vbADC = FollowViewBox()
        self.plotADC = pg.PlotWidget(viewBox=self.vbADC)
        self.plotADC.showGrid(x=True, y=True)
        self.plotADC.setLabel('bottom', 'Muestras', units='')
        self.plotADC.setYRange(0, 3.3)
        self.plotADC.setTitle("Entradas Analógicas")
        self.plotADC.setLabel('left', ' Voltaje', units='V')
        self.curves_adc = [
        self.plotADC.plot(pen=pg.mkPen('y', width=1.5)),   # ADC0 → Amarillo
        self.plotADC.plot(pen=pg.mkPen('g', width=1.5)),   # ADC1 → Verde
        self.plotADC.plot(pen=pg.mkPen('r', width=1.5)),   # ADC2 → Rojo
        self.plotADC.plot(pen=pg.mkPen('c', width=1.5))    # ADC3 → Cian
        ]
        
        # Leyendas de curvas en la esquina superior derecha
        self.legendADC = self.plotADC.addLegend(offset=(-10, 10))  # dentro del área del gráfico
        self.legendADC.setBrush(pg.mkBrush(0, 0, 0, 150))          # fondo semitransparente
        self.legendADC.anchor((1, 0), (1, 0), offset=(-10, 10))    # esquina superior derecha

        # Asociación de nombres y curvas
        self.legendADC.addItem(self.curves_adc[0], "CH1")
        self.legendADC.addItem(self.curves_adc[1], "CH2")
        self.legendADC.addItem(self.curves_adc[2], "CH3")
        self.legendADC.addItem(self.curves_adc[3], "CH4")


        # DAC
        self.vbDAC = FollowViewBox()
        self.plotDAC = pg.PlotWidget(viewBox=self.vbDAC)
        self.plotDAC.showGrid(x=True, y=True)
        self.plotDAC.setLabel('bottom', 'Muestras', units='')
        self.plotDAC.setYRange(0, 3.3)
        self.plotDAC.setTitle("Salida Analógica")
        self.plotDAC.setLabel('left', 'Voltaje', units='V')
        self.curve_dac = self.plotDAC.plot(pen='y')

        self.max_points = 50000
        self.buf_adc = [collections.deque(maxlen=self.max_points) for _ in range(4)]
        self.buf_dac = collections.deque(maxlen=self.max_points)

        self.lab_adc_vals = [value_label(f"CH{i+1}: ", center=True) for i in range(4)]
        self.lab_dac_val  = value_label("DAC: ", center=True)

        # XY dentro del marco
        analog_grid = QtWidgets.QGridLayout()
        analog_grid.setHorizontalSpacing(18); analog_grid.setVerticalSpacing(10)
        analog_grid.addWidget(self.plotADC, 0, 0)
        analog_grid.addWidget(self.plotDAC, 0, 1)

        adc_vals_row = QtWidgets.QHBoxLayout(); adc_vals_row.setSpacing(20); adc_vals_row.setAlignment(Qt.AlignHCenter)
        for lab in self.lab_adc_vals: adc_vals_row.addWidget(lab)
        dac_vals_row = QtWidgets.QHBoxLayout(); dac_vals_row.setSpacing(20); dac_vals_row.setAlignment(Qt.AlignHCenter)
        dac_vals_row.addWidget(self.lab_dac_val)

        analog_grid.addLayout(adc_vals_row, 1, 0)
        analog_grid.addLayout(dac_vals_row, 1, 1)

        analog_inner = QtWidgets.QWidget(); analog_inner.setLayout(analog_grid)
        analog_group = make_frame_with_title("Canales Analógicos", analog_inner)

        # ---------- Vistas 3D ----------
        self.viewAcc = self._make_3d_view()
        self.viewGyr = self._make_3d_view()
        self.viewRPY = self._make_3d_view()
        self.viewAcc.setMinimumHeight(360)
        self.viewGyr.setMinimumHeight(360)
        self.viewRPY.setMinimumHeight(360)
        self.viewAcc.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.viewGyr.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.viewRPY.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)

        self.arrowAcc = Arrow3D(color=(0,1,0,1))
        self.arrowGyr = Arrow3D(color=(1,0,0,1))
        self.arrowRPY = Arrow3D(color=(0.2,0.6,1.0,1))
        self.viewAcc.addItem(self.arrowAcc)
        self.viewGyr.addItem(self.arrowGyr)
        self.viewRPY.addItem(self.arrowRPY)

        self.lab_acc_xyz = [value_label("x : ", True), value_label("y : ", True), value_label("z : ", True)]
        self.lab_gyr_xyz = [value_label("x : ", True), value_label("y : ", True), value_label("z : ", True)]
        self.lab_rpy_xyz = [value_label("Roll : ", True), value_label("Pitch : ", True), value_label("Yaw : ", True)]

        def pack_3d(title, glw, labs):
            w = QtWidgets.QWidget()
            v = QtWidgets.QVBoxLayout(w); v.setContentsMargins(0,0,0,0); v.setSpacing(6)
            v.addWidget(legend_label(title))
            v.addWidget(glw, 1)  # ocupa el máximo de ventana
            row = QtWidgets.QHBoxLayout(); row.setSpacing(20); row.setAlignment(Qt.AlignHCenter)
            for l in labs: row.addWidget(l)
            v.addLayout(row)
            return w

        accW = pack_3d("Aceleración [m/s²]", self.viewAcc, self.lab_acc_xyz)
        gyrW = pack_3d("Velocidad angular [°/s]", self.viewGyr, self.lab_gyr_xyz)
        rpyW = pack_3d("Roll, Pitch, Yaw [°]", self.viewRPY, self.lab_rpy_xyz)

        inertial_row = QtWidgets.QHBoxLayout()
        inertial_row.setSpacing(12)
        inertial_row.addWidget(accW); inertial_row.setStretchFactor(accW, 1)
        inertial_row.addWidget(gyrW); inertial_row.setStretchFactor(gyrW, 1)
        inertial_row.addWidget(rpyW); inertial_row.setStretchFactor(rpyW, 1)

        inertial_inner = QtWidgets.QWidget(); inertial_inner.setLayout(inertial_row)
        inertial_group = make_frame_with_title("Datos de movimiento inercial", inertial_inner)

        # ---------- Registro ----------
        self.logEdit = QtWidgets.QPlainTextEdit(); self.logEdit.setReadOnly(True)

        # ---------- Layout principal ----------
        lay = QtWidgets.QVBoxLayout(cw)
        lay.addLayout(top)
        lay.addWidget(analog_group)
        lay.addWidget(inertial_group, 1)  # prioridad vertical a los 3D
        lay.addWidget(QtWidgets.QLabel("Registro"))
        lay.addWidget(self.logEdit, 1)

        # ---------- Serial ----------
        self.serial = QSerialPort(readyRead=self.on_ready_read)
        self.rxbuf = bytearray()
        self.current_mode = None
        self._roll = 0.0; self._pitch = 0.0; self._yaw = 0.0

        # Timer
        self.timer = QtCore.QTimer(interval=33, timeout=self.refresh_plots)
        self.timer.start()

        # Señal botón seguir
        self.btnFollow.toggled.connect(self._on_follow_toggled)

    # ---- 3D view base ----
    def _make_3d_view(self):
        v = GLViewWidget()
        v.opts['distance'] = 120
        gxy = GLGridItem(); gxy.setSize(60,60); gxy.setSpacing(10,10); v.addItem(gxy)
        gxz = GLGridItem(); gxz.rotate(90,1,0,0); gxz.translate(0,0,-50); gxz.setSize(60,60); gxz.setSpacing(10,10); v.addItem(gxz)
        gyz = GLGridItem(); gyz.rotate(90,0,1,0); gyz.translate(-50,0,0); gyz.setSize(60,60); gyz.setSpacing(10,10); v.addItem(gyz)
        v.opts['center'] = QVector3D(25,25,25)
        return v

    # ---- puertos ----
    def refresh_ports(self):
        cur = self.portBox.currentText()
        self.portBox.clear()
        for p in QSerialPortInfo.availablePorts():
            self.portBox.addItem(p.portName())
        if cur:
            idx = self.portBox.findText(cur)
            if idx >= 0: self.portBox.setCurrentIndex(idx)

    def toggle_port(self):
        if self.serial.isOpen():
            self.serial.close(); self.btnConn.setText("Conectar")
            return
        port = self.portBox.currentText()
        if not port:
            QtWidgets.QMessageBox.warning(self,"Puerto","No hay puerto seleccionado."); return
        self.serial.setPortName(port)
        self.serial.setBaudRate(int(self.baudBox.currentText()))
        self.serial.setDataBits(QSerialPort.Data8)
        self.serial.setParity(QSerialPort.NoParity)
        self.serial.setStopBits(QSerialPort.OneStop)
        self.serial.setFlowControl(QSerialPort.NoFlowControl)
        if not self.serial.open(QtCore.QIODevice.ReadWrite):
            QtWidgets.QMessageBox.critical(self,"Error","No se pudo abrir el puerto."); return
        self.btnConn.setText("Desconectar")

        # Reset buffers y vista
        for i in range(4): self.buf_adc[i].clear()
        self.buf_dac.clear()
        self.vbADC.follow = True
        self.vbDAC.follow = True
        self.btnFollow.setChecked(True)
        self.plotADC.enableAutoRange(x=False, y=False)
        self.plotDAC.enableAutoRange(x=False, y=False)
        self.plotADC.setXRange(0, WIN_POINTS, padding=0)
        self.plotDAC.setXRange(0, WIN_POINTS, padding=0)

    def _on_follow_toggled(self, checked: bool):
        self.vbADC.follow = checked
        self.vbDAC.follow = checked
        if checked:
            # Reenganchar muestra actual en X
            nA = max(len(b) for b in self.buf_adc) if self.buf_adc else 0
            nD = len(self.buf_dac)
            if nA <= WIN_POINTS:
                self.plotADC.setXRange(0, WIN_POINTS, padding=0)
            else:
                self.plotADC.setXRange(nA - WIN_POINTS, nA, padding=0)
            if nD <= WIN_POINTS:
                self.plotDAC.setXRange(0, WIN_POINTS, padding=0)
            else:
                self.plotDAC.setXRange(nD - WIN_POINTS, nD, padding=0)

    # ---- lectura serie ----
    def on_ready_read(self):
        self.rxbuf += self.serial.readAll().data()
        while True:
            if not self.rxbuf: return
            hdr = self.rxbuf[0]
            if hdr not in PROTO:
                pos = next((i for i,b in enumerate(self.rxbuf) if b in PROTO), -1)
                if pos < 0: self.rxbuf.clear(); return
                if pos > 0: del self.rxbuf[:pos]
                hdr = self.rxbuf[0]
            cfg = PROTO[hdr]
            if len(self.rxbuf) < cfg["frame_len"]: return
            frame = bytes(self.rxbuf[:cfg["frame_len"]])
            del self.rxbuf[:cfg["frame_len"]]
            self.process_frame_by_mode(hdr, frame)

    @staticmethod
    def _int16_from_bytes(lsb, msb):
        u = (msb << 8) | lsb
        return u - 65536 if u >= 32768 else u

    def process_frame_by_mode(self, hdr: int, frame: bytes):
        cfg = PROTO[hdr]

        if self.current_mode != hdr:
            self.current_mode = hdr
            self.modeBox.setText(cfg["name"])

        # CRC bilateral (acepta H,L o L,H)
        crc_off = cfg["OFF"]["CRC"]
        recv_h = frame[crc_off]; recv_l = frame[crc_off+1]
        calc = crc16_modbus(frame[:crc_off])
        ok = ((recv_h == ((calc>>8)&0xFF) and recv_l == (calc&0xFF)) or
              (recv_l == ((calc>>8)&0xFF) and recv_h == (calc&0xFF)))
        self.serial.write(ACK if ok else NACK)
        if not ok: return

        def get16s(off): return self._int16_from_bytes(frame[off], frame[off+1])
        def u16_le(off): return ((frame[off+1] << 8) | frame[off]) & 0xFFFF

        # Variables para registro
        adc0 = adc1 = adc2 = adc3 = dac = None
        Ax_raw = Ay_raw = Az_raw = Gx_raw = Gy_raw = Gz_raw = None
        Ax = Ay = Az = Gx = Gy = Gz = None

        # ADC/DAC
        if cfg["have_dac"]:
            dac = frame[cfg["OFF"]["DAC"]]
            self.buf_dac.append(dac)
            self.lab_dac_val.setText(f"DAC: {dac}")
        else:
            # Empujar "muestra vacía" para que X avance sin dibujar cuando se cambia de modo
            self.buf_dac.append(np.nan)

        if cfg["have_adc"]:
            adc0 = u16_le(cfg["OFF"]["ADC0"]); adc1 = u16_le(cfg["OFF"]["ADC1"])
            adc2 = u16_le(cfg["OFF"]["ADC2"]); adc3 = u16_le(cfg["OFF"]["ADC3"])
            self.buf_adc[0].append(adc0); self.buf_adc[1].append(adc1)
            self.buf_adc[2].append(adc2); self.buf_adc[3].append(adc3)
            self.lab_adc_vals[0].setText(f"CH1: {adc0}")
            self.lab_adc_vals[1].setText(f"CH2: {adc1}")
            self.lab_adc_vals[2].setText(f"CH3: {adc2}")
            self.lab_adc_vals[3].setText(f"CH4: {adc3}")
        else:
            for i in range(4):
                self.buf_adc[i].append(np.nan)

        # IMU
        if cfg["have_imu"]:
            Ax_raw = get16s(cfg["OFF"]["AX"]); Ay_raw = get16s(cfg["OFF"]["AY"]); Az_raw = get16s(cfg["OFF"]["AZ"])
            Gx_raw = get16s(cfg["OFF"]["GX"]); Gy_raw = get16s(cfg["OFF"]["GY"]); Gz_raw = get16s(cfg["OFF"]["GZ"])

            Ax = (Ax_raw / ACCEL_SCALE) * GRAVITY
            Ay = (Ay_raw / ACCEL_SCALE) * GRAVITY
            Az = (Az_raw / ACCEL_SCALE) * GRAVITY
            Gx = (Gx_raw / GYRO_SCALE)  # °/s
            Gy = (Gy_raw / GYRO_SCALE)
            Gz = (Gz_raw / GYRO_SCALE)

            # Complementario para RPY (en grados)
            accel_roll  = math.degrees(math.atan2(Ay, Az))
            accel_pitch = math.degrees(math.atan2(-Ax, math.sqrt(Ay*Ay + Az*Az)))
            self._roll  = ALPHA*(self._roll  + Gx*DT) + (1-ALPHA)*accel_roll
            self._pitch = ALPHA*(self._pitch + Gy*DT) + (1-ALPHA)*accel_pitch
            self._yaw   = self._yaw + Gz*DT

            # Vector unitario para visualizar RPY (flecha longitud 50)
            ux, uy, uz = rpy_to_unit_vector(self._roll, self._pitch, self._yaw)
            self.arrowRPY.update_vec(50.0*ux, 50.0*uy, 50.0*uz)

            # Aceleración / Giro
            clamp = lambda v: max(-50, min(50, v))
            self.arrowAcc.update_vec(
                clamp(Ax * (50.0 / MAX_ACC)),
                clamp(Ay * (50.0 / MAX_ACC)),
                clamp(Az * (50.0 / MAX_ACC))
            )
            self.arrowGyr.update_vec(
                clamp(Gx * (50.0 / MAX_GYR)),
                clamp(Gy * (50.0 / MAX_GYR)),
                clamp(Gz * (50.0 / MAX_GYR))
            )

            # Etiquetas
            self.lab_acc_xyz[0].setText(f"x : {Ax:+.2f}")
            self.lab_acc_xyz[1].setText(f"y : {Ay:+.2f}")
            self.lab_acc_xyz[2].setText(f"z : {Az:+.2f}")
            self.lab_gyr_xyz[0].setText(f"x : {Gx:+.2f}")
            self.lab_gyr_xyz[1].setText(f"y : {Gy:+.2f}")
            self.lab_gyr_xyz[2].setText(f"z : {Gz:+.2f}")

            def wrap180(a): return (a + 180.0) % 360.0 - 180.0
            self.lab_rpy_xyz[0].setText(f"Roll : {wrap180(self._roll):+.1f}°")
            self.lab_rpy_xyz[1].setText(f"Pitch : {wrap180(self._pitch):+.1f}°")
            self.lab_rpy_xyz[2].setText(f"Yaw : {wrap180(self._yaw):+.1f}°")

        # -------- Registro detallado --------
        ts = time.strftime("%H:%M:%S")
        parts = [f"{ts} - "]
        if cfg["have_adc"]:
            parts.append(f"ADC0:{adc0} ADC1:{adc1} ADC2:{adc2} ADC3:{adc3} ")
        else:
            parts.append("ADC0:— ADC1:— ADC2:— ADC3:— ")
        parts.append(f"DAC:{dac if dac is not None else '—'} ")

        if cfg["have_imu"]:
            parts.append(f"Ax_raw:{Ax_raw} Ay_raw:{Ay_raw} Az_raw:{Az_raw} "
                         f"Gx_raw:{Gx_raw} Gy_raw:{Gy_raw} Gz_raw:{Gz_raw} | ")
            parts.append(f"A[g]:{(Ax/GRAVITY):+.2f} {(Ay/GRAVITY):+.2f} {(Az/GRAVITY):+.2f} "
                         f"G[°/s]:{Gx:+.2f} {Gy:+.2f} {Gz:+.2f} | ")
            parts.append(f"Roll:{self._roll:+.1f}° Pitch:{self._pitch:+.1f}° Yaw:{self._yaw:+.1f}°")
        else:
            parts.append("Ax_raw:— Ay_raw:— Az_raw:— Gx_raw:— Gy_raw:— Gz_raw:—")

        self.logEdit.appendPlainText("".join(parts))

    # ---- refresco XY  ----
    def refresh_plots(self):
        # ADC
        for i, c in enumerate(self.curves_adc):
            n = len(self.buf_adc[i])
            if n:
                x = np.arange(n, dtype=float)
                y = np.fromiter((v * 3.3 / 4095.0 for v in self.buf_adc[i]), dtype=float, count=n)
                c.setData(x, y)
        # DAC
        nd = len(self.buf_dac)
        if nd:
            xd = np.arange(nd, dtype=float)
            yd = np.fromiter((v * 3.3 / 255.0 for v in self.buf_dac), dtype=float, count=nd)
            self.curve_dac.setData(xd, yd)

        # Rango X: sólo si 'seguir' está activo
        if self.vbADC.follow:
            nA = max((len(b) for b in self.buf_adc), default=0)
            if nA <= WIN_POINTS:
                self.plotADC.setXRange(0, WIN_POINTS, padding=0)
            else:
                self.plotADC.setXRange(nA - WIN_POINTS, nA, padding=0)
        if self.vbDAC.follow:
            nD = len(self.buf_dac)
            if nD <= WIN_POINTS:
                self.plotDAC.setXRange(0, WIN_POINTS, padding=0)
            else:
                self.plotDAC.setXRange(nD - WIN_POINTS, nD, padding=0)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    w = MainWindow(); w.resize(1280, 900); w.show()
    sys.exit(app.exec_())

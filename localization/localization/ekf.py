import numpy as np
import math
from threading import Lock, RLock
from typing import Tuple

# Constants
# 노이즈 표준편차
SIG_W_A = 0.05 # 가속도계 / 백색 노이즈 / 표준편차 (m/s^2)
SIG_A_D = 0.01 # 가속도계 / 드리프트 노이즈 / 표준편차 (m/s^2)
TAU_A = 100.0 # 가속도계 / 드리프트 노이즈 / 시간 상수 (s)
SIG_W_G = 0.00175 # 자이로스코프 / 백색 노이즈 / 표준편차 (rad/s)
SIG_G_D = 0.00025 # 자이로스코프 / 드리프트 노이즈 / 표준편차 (rad/s)
TAU_G = 50.0 # 자이로스코프 / 드리프트 노이즈 / 시간 상수 (s)
SIG_GPS_P_NE = 3.0 # GPS 위치 (x,y) / 백색 노이즈 / 표준편차 (m)
SIG_GPS_P_D = 6.0 # GPS 위치 (z) / 백색 노이즈 / 표준편차 (m)
SIG_GPS_V_NE = 0.5 # GPS 속도 (x,y) / 백색 노이즈 / 표준편차 (m/s)
SIG_GPS_V_D = 1.0 # GPS 속도 (z) / 백색 노이즈 / 표준편차 (m/s)
# 초기 공분산
P_P_INIT = 10.0 # 위치 / 초기 공분산 (m^2)
P_V_INIT = 1.0 # 속도 / 초기 공분산 (m^2/s^2)
P_A_INIT = 0.34906 # 자세 / 초기 공분산 (rad^2)
P_HDG_INIT = 3.14159 # 헤딩 / 초기 공분산 (rad^2)
P_AB_INIT = 0.9810 # 가속도계 / 초기 공분산 (m/s^2)
P_GB_INIT = 0.01745 # 자이로스코프 / 초기 공분산 (rad/s)

# 물리적 상수
G = 9.807 # 중력 가속도 (m/s^2)
ECC2 = 0.0066943799901 # 지구 타원체의 제곱 편심률
EARTH_RADIUS = 6378137.0 # 지구의 반지름 (m)

class GPSCoordinate:
    def __init__(self, lat: float, lon: float, alt: float):
        self.lat = lat
        self.lon = lon
        self.alt = alt

class GPSVelocity:
    def __init__(self, vN: float, vE: float, vD: float):
        self.vN = vN
        self.vE = vE
        self.vD = vD

class IMUData:
    def __init__(self, gyroX: float, gyroY: float, gyroZ: float, accX: float, accY: float, accZ: float, hX: float, hY: float, hZ: float):
        self.gyroX = gyroX
        self.gyroY = gyroY
        self.gyroZ = gyroZ
        self.accX = accX
        self.accY = accY
        self.accZ = accZ
        self.hX = hX
        self.hY = hY
        self.hZ = hZ

class EKFNavINS:
    def __init__(self):
        self.gpsCoor = GPSCoordinate(0, 0, 0)  # GPS 좌표 (위도, 경도, 고도)
        self.gpsVel = GPSVelocity(0, 0, 0)  # GPS 속도 (북쪽, 동쪽, 아래쪽)
        self.imuDat = IMUData(0, 0, 0, 0, 0, 0, 0, 0, 0)  # IMU 데이터 (자이로스코프, 가속도계, 자기장)
        self.shMutex = RLock()  # 스레드 안전을 위한 재진입 가능 락
        self.initialized_ = False  # EKF 초기화 여부
        self._tprev = 0  # 이전 시간
        self.phi = self.theta = self.psi = 0.0  # 오일러 각도 (롤, 피치, 요)
        self.vn_ins = self.ve_ins = self.vd_ins = 0.0  # INS 속도 (북쪽, 동쪽, 아래쪽)
        self.lat_ins = self.lon_ins = self.alt_ins = 0.0  # INS 위치 (위도, 경도, 고도)
        self.Bxc = self.Byc = 0.0  # 자기장 보정 값
        self.abx = self.aby = self.abz = 0.0  # 가속도계 바이어스
        self.gbx = self.gby = self.gbz = 0.0  # 자이로스코프 바이어스
        self.Fs = np.identity(15, dtype=float)  # 상태 전이 행렬
        self.PHI = np.zeros((15, 15), dtype=float)  # 상태 전이 행렬의 근사치
        self.P = np.zeros((15, 15), dtype=float)  # 상태 공분산 행렬
        self.Gs = np.zeros((15, 12), dtype=float)  # 프로세스 노이즈 행렬
        self.Rw = np.zeros((12, 12), dtype=float)  # 프로세스 노이즈 공분산 행렬
        self.Q = np.zeros((15, 15), dtype=float)  # 프로세스 노이즈 공분산 행렬의 근사치
        self.grav = np.zeros((3, 1), dtype=float)  # 중력 벡터
        self.om_ib = np.zeros((3, 1), dtype=float)  # 자이로스코프 측정 값 (각속도)
        self.f_b = np.zeros((3, 1), dtype=float)  # 가속도계 측정 값 (가속도)
        self.C_N2B = np.zeros((3, 3), dtype=float)  # 네비게이션 좌표계에서 바디 좌표계로의 변환 행렬
        self.C_B2N = np.zeros((3, 3), dtype=float)  # 바디 좌표계에서 네비게이션 좌표계로의 변환 행렬
        self.dx = np.zeros((3, 1), dtype=float)  # 상태 변화 벡터
        self.dxd = np.zeros((3, 1), dtype=float)  # 상태 변화 벡터의 근사치
        self.estmimated_ins = np.zeros((3, 1), dtype=float)  # 추정된 INS 상태
        self.V_ins = np.zeros((3, 1), dtype=float)  # INS 속도 벡터
        self.lla_ins = np.zeros((3, 1), dtype=float)  # INS 위치 벡터 (위도, 경도, 고도)
        self.V_gps = np.zeros((3, 1), dtype=float)  # GPS 속도 벡터
        self.lla_gps = np.zeros((3, 1), dtype=float)  # GPS 위치 벡터 (위도, 경도, 고도)
        self.pos_ecef_ins = np.zeros((3, 1), dtype=float)  # INS 위치 (ECEF 좌표계)
        self.pos_ned_ins = np.zeros((3, 1), dtype=float)  # INS 위치 (NED 좌표계)
        self.pos_ecef_gps = np.zeros((3, 1), dtype=float)  # GPS 위치 (ECEF 좌표계)
        self.pos_ned_gps = np.zeros((3, 1), dtype=float)  # GPS 위치 (NED 좌표계)
        self.quat = np.zeros((4, 1), dtype=float)  # 쿼터니언 (자세)
        self.dq = np.zeros((4, 1), dtype=float)  # 쿼터니언 변화량
        self.y = np.zeros((6, 1), dtype=float)  # 측정 잔차 벡터
        self.R = np.zeros((6, 6), dtype=float)  # 측정 노이즈 공분산 행렬
        self.x = np.zeros((15, 1), dtype=float)  # 상태 벡터
        self.K = np.zeros((15, 6), dtype=float)  # 칼만 이득 행렬
        self.H = np.zeros((6, 15), dtype=float)  # 측정 행렬

    def ekf_init(self, time, vn, ve, vd, lat, lon, alt, p, q, r, ax, ay, az, hx, hy, hz):
        self.gbx = p  # 자이로스코프 바이어스 (x축)
        self.gby = q  # 자이로스코프 바이어스 (y축)
        self.gbz = r  # 자이로스코프 바이어스 (z축)
        self.theta, self.phi, self.psi = self.get_pitch_roll_yaw(ax, ay, az, hx, hy, hz)  # 오일러 각도 (피치, 롤, 요)
        self.quat = self.to_quaternion(self.phi, self.theta, self.psi)  # 쿼터니언 (자세)
        self.grav[2, 0] = G  # 중력 가속도 벡터
        self.H[:5, :5] = np.identity(5)  # 측정 행렬 초기화
        self.Rw[:3, :3] = (SIG_W_A ** 2) * np.identity(3)  # 가속도계 백색 노이즈 공분산 행렬
        self.Rw[3:6, 3:6] = (SIG_W_G ** 2) * np.identity(3)  # 자이로스코프 백색 노이즈 공분산 행렬
        self.Rw[6:9, 6:9] = (2 * (SIG_A_D ** 2) / TAU_A) * np.identity(3)  # 가속도계 드리프트 노이즈 공분산 행렬
        self.Rw[9:12, 9:12] = (2 * (SIG_G_D ** 2) / TAU_G) * np.identity(3)  # 자이로스코프 드리프트 노이즈 공분산 행렬
        self.P[:3, :3] = (P_P_INIT ** 2) * np.identity(3)  # 초기 위치 공분산 행렬
        self.P[3:6, 3:6] = (P_V_INIT ** 2) * np.identity(3)  # 초기 속도 공분산 행렬
        self.P[6:8, 6:8] = (P_A_INIT ** 2) * np.identity(2)  # 초기 자세 공분산 행렬
        self.P[8, 8] = P_HDG_INIT ** 2  # 초기 헤딩 공분산 행렬
        self.P[9:12, 9:12] = (P_AB_INIT ** 2) * np.identity(3)  # 초기 가속도계 바이어스 공분산 행렬
        self.P[12:15, 12:15] = (P_GB_INIT ** 2) * np.identity(3)  # 초기 자이로스코프 바이어스 공분산 행렬
        self.R[:2, :2] = (SIG_GPS_P_NE ** 2) * np.identity(2)  # GPS 위치 노이즈 공분산 행렬 (북쪽, 동쪽)
        self.R[2, 2] = SIG_GPS_P_D ** 2  # GPS 위치 노이즈 공분산 행렬 (수직)
        self.R[3:5, 3:5] = (SIG_GPS_V_NE ** 2) * np.identity(2)  # GPS 속도 노이즈 공분산 행렬 (북쪽, 동쪽)
        self.R[5, 5] = SIG_GPS_V_D ** 2  # GPS 속도 노이즈 공분산 행렬 (수직)
        self.lat_ins = lat  # 초기 INS 위도
        self.lon_ins = lon  # 초기 INS 경도
        self.alt_ins = alt  # 초기 INS 고도
        self.vn_ins = vn  # 초기 INS 북쪽 속도
        self.ve_ins = ve  # 초기 INS 동쪽 속도
        self.vd_ins = vd  # 초기 INS 수직 속도
        self.f_b[0, 0] = ax  # 가속도계 측정 값 (x축)
        self.f_b[1, 0] = ay  # 가속도계 측정 값 (y축)
        self.f_b[2, 0] = az  # 가속도계 측정 값 (z축)
        self._tprev = time  # 이전 시간

    def ekf_update(self, time, vn=None, ve=None, vd=None, lat=None, lon=None, alt=None, p=None, q=None, r=None, ax=None, ay=None, az=None, hx=None, hy=None, hz=None):
        if not self.initialized_:
            # EKF 초기화
            self.ekf_init(time, vn, ve, vd, lat, lon, alt, p, q, r, ax, ay, az, hx, hy, hz)
            self.initialized_ = True
        else:
            # 시간 간격 계산
            _dt = (time - self._tprev) / 1e6
            
            # 바이어스 업데이트
            self.update_bias(ax, ay, az, p, q, r)
            
            # INS 업데이트
            self.update_ins()
            
            # 쿼터니언 업데이트
            self.dq[0, 0] = 1.0
            self.dq[1, 0] = 0.5 * self.om_ib[0, 0] * _dt
            self.dq[2, 0] = 0.5 * self.om_ib[1, 0] * _dt
            self.dq[3, 0] = 0.5 * self.om_ib[2, 0] * _dt
            self.quat = self.qmult(self.quat, self.dq)
            self.quat /= np.linalg.norm(self.quat)
            if self.quat[0, 0] < 0:
                self.quat *= -1.0
            
            # 변환 행렬 업데이트
            self.C_N2B = self.quat2dcm(self.quat)
            self.C_B2N = self.C_N2B.T
            
            # 오일러 각도 업데이트
            self.phi, self.theta, self.psi = self.to_euler_angles(self.quat)
            
            # 상태 변화 벡터 업데이트
            self.dx = self.C_B2N @ self.f_b + self.grav
            self.vn_ins += _dt * self.dx[0, 0]
            self.ve_ins += _dt * self.dx[1, 0]
            self.vd_ins += _dt * self.dx[2, 0]
            
            # 위치 변화 벡터 업데이트
            self.dxd = self.llarate(self.V_ins, self.lla_ins)
            self.lat_ins += _dt * self.dxd[0, 0]
            self.lon_ins += _dt * self.dxd[1, 0]
            self.alt_ins += _dt * self.dxd[2, 0]
            
            # 자코비안 행렬 업데이트
            self.update_jacobian_matrix()
            
            # 프로세스 노이즈 공분산 행렬 업데이트
            self.update_process_noise_covariance_time(_dt)
            
            if (time - self._tprev) > 0:
                # GPS 데이터 업데이트
                self.lla_gps[0, 0] = lat
                self.lla_gps[1, 0] = lon
                self.lla_gps[2, 0] = alt
                self.V_gps[0, 0] = vn
                self.V_gps[1, 0] = ve
                self.V_gps[2, 0] = vd
                
                # INS 업데이트
                self.update_ins()
                
                # 측정 잔차 벡터 업데이트
                self.update_calculated_vs_predicted()
                
                # 칼만 이득 계산
                self.K = self.P @ self.H.T @ np.linalg.inv(self.H @ self.P @ self.H.T + self.R)
                
                # 상태 공분산 행렬 업데이트
                self.P = (np.identity(15) - self.K @ self.H) @ self.P @ (np.identity(15) - self.K @ self.H).T + self.K @ self.R @ self.K.T
                
                # 상태 벡터 업데이트
                self.x = self.K @ self.y
                
                # 상태 업데이트
                self.update_15states_after_KF()
                
                # 이전 시간 업데이트
                self._tprev = time
            
            # 바이어스 업데이트
            self.update_bias(ax, ay, az, p, q, r)

    def imu_update_ekf(self, time, imu):
        with Lock():
            self.imuDat = imu
        self.ekf_update(time)

    def gps_coordinate_update_ekf(self, coor):
        with Lock():
            self.gpsCoor = coor

    def gps_velocity_update_ekf(self, vel):
        with Lock():
            self.gpsVel = vel

    def update_ins(self):
        self.lla_ins[0, 0] = self.lat_ins
        self.lla_ins[1, 0] = self.lon_ins
        self.lla_ins[2, 0] = self.alt_ins
        self.V_ins[0, 0] = self.vn_ins
        self.V_ins[1, 0] = self.ve_ins
        self.V_ins[2, 0] = self.vd_ins

    def get_pitch_roll_yaw(self, ax, ay, az, hx, hy, hz):
        theta = math.asin(ax / G)
        phi = -math.asin(ay / (G * math.cos(theta)))
        Bxc = hx * math.cos(theta) + (hy * math.sin(phi) + hz * math.cos(phi)) * math.sin(theta)
        Byc = hy * math.cos(phi) - hz * math.sin(phi)
        psi = -math.atan2(Byc, Bxc)
        return theta, phi, psi

    def update_calculated_vs_predicted(self):
        self.pos_ecef_ins = self.lla2ecef(self.lla_ins)
        self.pos_ecef_gps = self.lla2ecef(self.lla_gps)
        self.pos_ned_gps = self.ecef2ned(self.pos_ecef_gps - self.pos_ecef_ins, self.lla_ins)
        self.y[0, 0] = self.pos_ned_gps[0, 0]
        self.y[1, 0] = self.pos_ned_gps[1, 0]
        self.y[2, 0] = self.pos_ned_gps[2, 0]
        self.y[3, 0] = self.V_gps[0, 0] - self.V_ins[0, 0]
        self.y[4, 0] = self.V_gps[1, 0] - self.V_ins[1, 0]
        self.y[5, 0] = self.V_gps[2, 0] - self.V_ins[2, 0]

    def update_15states_after_KF(self):
        self.estmimated_ins = self.llarate(self.x[:3].astype(float), self.lat_ins, self.alt_ins)
        self.lat_ins += self.estmimated_ins[0, 0]
        self.lon_ins += self.estmimated_ins[1, 0]
        self.alt_ins += self.estmimated_ins[2, 0]
        self.vn_ins += self.x[3, 0]
        self.ve_ins += self.x[4, 0]
        self.vd_ins += self.x[5, 0]
        self.dq[0, 0] = 1.0
        self.dq[1, 0] = self.x[6, 0]
        self.dq[2, 0] = self.x[7, 0]
        self.dq[3, 0] = self.x[8, 0]
        self.quat = self.qmult(self.quat, self.dq)
        self.quat /= np.linalg.norm(self.quat)
        self.phi, self.theta, self.psi = self.to_euler_angles(self.quat)
        self.abx += self.x[9, 0]
        self.aby += self.x[10, 0]
        self.abz += self.x[11, 0]
        self.gbx += self.x[12, 0]
        self.gby += self.x[13, 0]
        self.gbz += self.x[14, 0]

    def update_bias(self, ax, ay, az, p, q, r):
        self.f_b[0, 0] = ax - self.abx
        self.f_b[1, 0] = ay - self.aby
        self.f_b[2, 0] = az - self.abz
        self.om_ib[0, 0] = p - self.gbx
        self.om_ib[1, 0] = q - self.gby
        self.om_ib[2, 0] = r - self.gbz

    def update_process_noise_covariance_time(self, _dt):
        self.PHI = np.identity(15) + self.Fs * _dt
        self.Gs.fill(0)
        self.Gs[3:6, :3] = -self.C_B2N
        self.Gs[6:9, 3:6] = -0.5 * np.identity(3)
        self.Gs[9:15, 6:12] = np.identity(6)
        self.Q = self.PHI * _dt * self.Gs @ self.Rw @ self.Gs.T
        self.Q = 0.5 * (self.Q + self.Q.T)
        self.P = self.PHI @ self.P @ self.PHI.T + self.Q
        self.P = 0.5 * (self.P + self.P.T)

    def update_jacobian_matrix(self):
        self.Fs.fill(0)
        self.Fs[:3, 3:6] = np.identity(3)
        self.Fs[5, 2] = -2.0 * G / EARTH_RADIUS
        self.Fs[3:6, 6:9] = -2.0 * self.C_B2N @ self.sk(self.f_b)
        self.Fs[3:6, 9:12] = -self.C_B2N
        self.Fs[6:9, 6:9] = -self.sk(self.om_ib)
        self.Fs[6:9, 12:15] = -0.5 * np.identity(3)
        self.Fs[9:12, 9:12] = -np.identity(3) / TAU_A
        self.Fs[12:15, 12:15] = -np.identity(3) / TAU_G

    def sk(self, w):
        C = np.zeros((3, 3), dtype=float)
        C[0, 1:3] = [-w[2, 0], w[1, 0]]
        C[1, [0, 2]] = [w[2, 0], -w[0, 0]]
        C[2, :2] = [-w[1, 0], w[0, 0]]
        return C

    def earthradius(self, lat):
        denom = abs(1.0 - (ECC2 * math.pow(math.sin(lat), 2.0)))
        Rew = EARTH_RADIUS / math.sqrt(denom)
        Rns = EARTH_RADIUS * (1.0 - ECC2) / (denom * math.sqrt(denom))
        return Rew, Rns

    def llarate_with_matrix(self, V, lla):
        Rew, Rns = self.earthradius(lla[0, 0])
        lla_dot = np.zeros((3, 1), dtype=float)
        lla_dot[0, 0] = V[0, 0] / (Rns + lla[2, 0])
        lla_dot[1, 0] = V[1, 0] / ((Rew + lla[2, 0]) * math.cos(lla[0, 0]))
        lla_dot[2, 0] = -V[2, 0]
        return lla_dot

    def llarate_with_values(self, V, lat, alt):
        lla = np.zeros((3, 1), dtype=float)
        lla[0, 0] = lat
        lla[1, 0] = 0.0
        lla[2, 0] = alt
        return self.llarate_with_matrix(V, lla)

    def lla2ecef(self, lla):
        Rew, _ = self.earthradius(lla[0, 0])
        ecef = np.zeros((3, 1), dtype=float)
        ecef[0, 0] = (Rew + lla[2, 0]) * math.cos(lla[0, 0]) * math.cos(lla[1, 0])
        ecef[1, 0] = (Rew + lla[2, 0]) * math.cos(lla[0, 0]) * math.sin(lla[1, 0])
        ecef[2, 0] = (Rew * (1.0 - ECC2) + lla[2, 0]) * math.sin(lla[0, 0])
        return ecef

    def ecef2ned(self, ecef, pos_ref):
        ned = np.zeros((3, 1), dtype=float)
        ned[1, 0] = -math.sin(pos_ref[1, 0]) * ecef[0, 0] + math.cos(pos_ref[1, 0]) * ecef[1, 0]
        ned[0, 0] = -math.sin(pos_ref[0, 0]) * math.cos(pos_ref[1, 0]) * ecef[0, 0] - math.sin(pos_ref[0, 0]) * math.sin(pos_ref[1, 0]) * ecef[1, 0] + math.cos(pos_ref[0, 0]) * ecef[2, 0]
        ned[2, 0] = -math.cos(pos_ref[0, 0]) * math.cos(pos_ref[1, 0]) * ecef[0, 0] - math.cos(pos_ref[0, 0]) * math.sin(pos_ref[1, 0]) * ecef[1, 0] - math.sin(pos_ref[0, 0]) * ecef[2, 0]
        return ned

    def quat2dcm(self, q):
        C_N2B = np.zeros((3, 3), dtype=float)
        C_N2B[0, 0] = 2.0 * q[0, 0]**2 - 1.0 + 2.0 * q[1, 0]**2
        C_N2B[1, 1] = 2.0 * q[0, 0]**2 - 1.0 + 2.0 * q[2, 0]**2
        C_N2B[2, 2] = 2.0 * q[0, 0]**2 - 1.0 + 2.0 * q[3, 0]**2

        C_N2B[0, 1] = 2.0 * q[1, 0] * q[2, 0] + 2.0 * q[0, 0] * q[3, 0]
        C_N2B[0, 2] = 2.0 * q[1, 0] * q[3, 0] - 2.0 * q[0, 0] * q[2, 0]

        C_N2B[1, 0] = 2.0 * q[1, 0] * q[2, 0] - 2.0 * q[0, 0] * q[3, 0]
        C_N2B[1, 2] = 2.0 * q[2, 0] * q[3, 0] + 2.0 * q[0, 0] * q[1, 0]

        C_N2B[2, 0] = 2.0 * q[1, 0] * q[3, 0] + 2.0 * q[0, 0] * q[2, 0]
        C_N2B[2, 1] = 2.0 * q[2, 0] * q[3, 0] - 2.0 * q[0, 0] * q[1, 0]
        return C_N2B

    def qmult(self, p, q):
        r = np.zeros((4, 1), dtype=float)
        r[0, 0] = p[0, 0] * q[0, 0] - (p[1, 0] * q[1, 0] + p[2, 0] * q[2, 0] + p[3, 0] * q[3, 0])
        r[1, 0] = p[0, 0] * q[1, 0] + q[0, 0] * p[1, 0] + p[2, 0] * q[3, 0] - p[3, 0] * q[2, 0]
        r[2, 0] = p[0, 0] * q[2, 0] + q[0, 0] * p[2, 0] + p[3, 0] * q[1, 0] - p[1, 0] * q[3, 0]
        r[3, 0] = p[0, 0] * q[3, 0] + q[0, 0] * p[3, 0] + p[1, 0] * q[2, 0] - p[2, 0] * q[1, 0]
        return r

    def constrain_angle180(self, dta):
        if dta > math.pi:
            dta -= 2.0 * math.pi
        if dta < -math.pi:
            dta += 2.0 * math.pi
        return dta

    def constrain_angle360(self, dta):
        dta = math.fmod(dta, 2.0 * math.pi)
        if dta < 0:
            dta += 2.0 * math.pi
        return dta

    def to_quaternion(self, yaw, pitch, roll):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        q = np.zeros((4, 1), dtype=float)
        q[0] = cr * cp * cy + sr * sp * sy  # w
        q[1] = cr * cp * sy - sr * sp * cy  # x
        q[2] = cr * sp * cy + sr * cp * sy  # y
        q[3] = sr * cp * cy - cr * sp * sy  # z
        return q

    def to_euler_angles(self, quat):
        roll = pitch = yaw = 0.0
        sinr_cosp = 2.0 * (quat[0, 0] * quat[1, 0] + quat[2, 0] * quat[3, 0])
        cosr_cosp = 1.0 - 2.0 * (quat[1, 0] * quat[1, 0] + quat[2, 0] * quat[2, 0])
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (quat[0, 0] * quat[2, 0] - quat[1, 0] * quat[3, 0])
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2.0, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (quat[1, 0] * quat[2, 0] + quat[0, 0] * quat[3, 0])
        cosy_cosp = 1.0 - 2.0 * (quat[2, 0] * quat[2, 0] + quat[3, 0] * quat[3, 0])
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw
#!/usr/bin/env python3

import numpy as np

class KalmanFilter:
    def __init__(self, F, B, H, Q, R, P, x):
        """
        F: 상태 전이 행렬
        B: 제어 입력 행렬
        H: 관측 행렬
        Q: 프로세스 노이즈 공분산
        R: 관측 노이즈 공분산
        P: 초기 공분산
        x: 초기 상태
        """
        self.F = F
        self.B = B
        self.H = H
        self.Q = Q
        self.R = R
        self.P = P
        self.x = x

    def predict(self, u):
        """
        예측 단계: 상태와 공분산을 업데이트
        u: 제어 입력
        """
        self.x = self.F @ self.x + self.B @ u
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        """
        갱신 단계: 관측값을 사용하여 상태와 공분산을 업데이트
        z: 관측값
        """
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        y = z - self.H @ self.x
        self.x = self.x + K @ y
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P

    def get_state(self):
        """
        현재 상태 반환
        """
        return self.x

def main():
    # 예제 초기 상태 및 행렬 정의
    F = np.array([[1, 1], [0, 1]])  # 상태 전이 행렬
    B = np.array([[0], [0]])        # 제어 입력 행렬
    H = np.array([[1, 0]])          # 관측 행렬
    Q = np.array([[1, 0], [0, 1]])  # 프로세스 노이즈 공분산
    R = np.array([[1]])             # 관측 노이즈 공분산
    P = np.array([[1, 0], [0, 1]])  # 초기 공분산
    x = np.array([0, 1])            # 초기 상태

    kf = KalmanFilter(F, B, H, Q, R, P, x)

    # 예측 단계
    u = np.array([0])  # 제어 입력
    kf.predict(u)

    # 갱신 단계
    z = np.array([1])  # 관측값
    kf.update(z)

    # 상태 출력
    print("Updated state:", kf.get_state())

if __name__ == '__main__':
    main()
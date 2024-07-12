import imageio
import cv2

# 비디오 캡처 객체 생성 (기본 카메라를 사용)
cap = cv2.VideoCapture(0)

# 해상도 설정 (카메라가 지원하는 해상도를 확인하고 맞게 설정)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# 녹화할 비디오 파일의 설정
writer = imageio.get_writer('output.mp4', fps=20)

while True:
    # 프레임 읽기
    ret, frame = cap.read()
    if not ret:
        break
    
    # 프레임의 색상 공간을 BGR에서 RGB로 변환 (imageio는 RGB 포맷을 사용)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # 프레임을 비디오 파일에 쓰기
    writer.append_data(rgb_frame)
    
    # 현재 프레임을 화면에 표시
    cv2.imshow('frame', frame)
    
    # 'q' 키를 누르면 녹화를 종료합니다.
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 모든 작업이 끝난 후 비디오 캡처 객체와 파일 쓰기 객체를 해제합니다.
cap.release()
writer.close()
cv2.destroyAllWindows()

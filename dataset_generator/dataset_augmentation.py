import cv2
import numpy as np
import random
from ultralytics import YOLO
import os
import time
from shutil import copyfile


def apply_various_sunlight_effects(image, num_effects=1):
    h, w = image.shape[:2]
    for _ in range(num_effects):
        # 사다리꼴의 꼭지점 좌표를 랜덤하게 생성
        x1, x2, x3, x4 = sorted(random.sample(range(w), 4))
        y1, y2 = random.sample(range(h), 2)

        pts = np.array([[x1, y1], [x2, y2], [x3, y2], [x4, y1]], np.int32)
        pts = pts.reshape((-1, 1, 2))

        mask = np.zeros((h, w), np.uint8)
        cv2.fillPoly(mask, [pts], (255))

        # 사다리꼴 영역 밝기 조절
        sunlight_intensity = random.uniform(0.2, 0.8)
        image_copy = image.copy()
        mask_indices = np.where(mask == 255)
        image_copy[mask_indices] = cv2.addWeighted(image[mask_indices], 1 - sunlight_intensity, np.full_like(image[mask_indices], (255, 255, 255)), sunlight_intensity, 0)

        image[mask_indices] = image_copy[mask_indices]

    return image


def apply_random_blur(image):
    if random.random() < 0.5:
        ksize = random.choice([(5, 5), (7, 7), (9, 9)])
        image = cv2.GaussianBlur(image, ksize, 0)
    return image


def apply_random_brightness_contrast(image):
    alpha = random.uniform(0.5, 1.5)
    beta = random.randint(-50, 50)
    image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    return image


def augment_image(image_path, num_sunlight_effects=2):
    image = cv2.imread(image_path)

    image = apply_various_sunlight_effects(image, num_sunlight_effects)
    image = apply_random_blur(image)
    image = apply_random_brightness_contrast(image)

    return image


# if __name__ == '__main__':
#     dataset_folders = ['./road_finder_dataset/train',
#                        './road_finder_dataset/valid',
#                        './road_finder_dataset/test']
#     num = 0
#     dataset_name = 'road_finder_augmented'
#     output_folder = time.strftime('%Y-%m-%d-%H%M%S', time.localtime()) + f'_{dataset_name}'
#
#     train_rate, valid_rate, test_rate = 0.8, 0.15, 0.05
#
#     os.mkdir(f'./{output_folder}')
#     os.mkdir(f'./{output_folder}/train')
#     os.mkdir(f'./{output_folder}/valid')
#     os.mkdir(f'./{output_folder}/test')
#     os.mkdir(f'./{output_folder}/train/images')
#     os.mkdir(f'./{output_folder}/valid/images')
#     os.mkdir(f'./{output_folder}/test/images')
#     os.mkdir(f'./{output_folder}/train/labels')
#     os.mkdir(f'./{output_folder}/valid/labels')
#     os.mkdir(f'./{output_folder}/test/labels')
#     result_names = []
#     for dataset_folder in dataset_folders:
#         for img_name in os.listdir(dataset_folder + '/images'):
#             image_path = f'{dataset_folder}/images/{img_name[:-4]}.jpg'
#             label_path = f'{dataset_folder}/labels/{img_name[:-4]}.txt'
#             img_aug1, img_aug2 = augment_image(image_path), augment_image(image_path)
#             copyfile(f'./{dataset_folder}/images/{img_name[:-4]}.jpg', f'./{output_folder}/train/images/{img_name[:-4]}_{num}.jpg')
#             copyfile(f'./{dataset_folder}/labels/{img_name[:-4]}.txt', f'./{output_folder}/train/labels/{img_name[:-4]}_{num}.txt')
#             result_names.append(f'{img_name[:-4]}_{num}')
#             num += 1
#             cv2.imwrite(f'./{output_folder}/train/images/{img_name[:-4]}_{num}.jpg', img_aug1)
#             copyfile(f'./{dataset_folder}/labels/{img_name[:-4]}.txt', f'./{output_folder}/train/labels/{img_name[:-4]}_{num}.txt')
#             result_names.append(f'{img_name[:-4]}_{num}')
#             num += 1
#             cv2.imwrite(f'./{output_folder}/train/images/{img_name[:-4]}_{num}.jpg', img_aug2)
#             copyfile(f'./{dataset_folder}/labels/{img_name[:-4]}.txt', f'./{output_folder}/train/labels/{img_name[:-4]}_{num}.txt')
#             result_names.append(f'{img_name[:-4]}_{num}')
#             num = 0
#
#     random.shuffle(result_names)
#     train_names = result_names[:int(train_rate * len(result_names))]
#     valid_names = result_names[int(train_rate * len(result_names)):int((train_rate + valid_rate) * len(result_names))]
#     test_names = result_names[int((train_rate + valid_rate) * len(result_names)):]
#     for name in valid_names:
#         copyfile(f'./{output_folder}/train/images/{name}.jpg',
#                  f'./{output_folder}/valid/images/{name}.jpg')
#         os.remove(f'./{output_folder}/train/images/{name}.jpg')
#         copyfile(f'./{output_folder}/train/labels/{name}.txt',
#                  f'./{output_folder}/valid/labels/{name}.txt')
#         os.remove(f'./{output_folder}/train/labels/{name}.txt')
#     for name in test_names:
#         copyfile(f'./{output_folder}/train/images/{name}.jpg',
#                  f'./{output_folder}/test/images/{name}.jpg')
#         os.remove(f'./{output_folder}/train/images/{name}.jpg')
#         copyfile(f'./{output_folder}/train/labels/{name}.txt',
#                  f'./{output_folder}/test/labels/{name}.txt')
#         os.remove(f'./{output_folder}/train/labels/{name}.txt')
#
#
#
#
# # 이미지 불러오기
image_path = './test_image.jpg'
image = cv2.imread(image_path)


model_1 = YOLO('../models/yolov8m-ep200-unf-d3.pt', task='segment')
model_2 = YOLO('../models/yolov8m-ep200-unf-d4.pt', task='segment')
model_1.to('mps')
model_2.to('mps')

while True:
    # 햇빛 효과 적용
    augmented_image = augment_image(image_path)

    # 결과 이미지 저장
    cv2.imwrite('augmented_image.jpg', augmented_image)
    results_1 = model_1(augmented_image, conf=0.4, device='mps')
    augmented_image_1 = results_1[0].plot()
    results_2 = model_2(augmented_image, conf=0.4, device='mps')
    augmented_image_2 = results_2[0].plot()
    # 결과 이미지 보기 (옵션)
    result = np.concatenate([augmented_image_1, augmented_image_2], axis=1)
    cv2.imshow('Sunlight Effect', result)
    cv2.waitKey(30)


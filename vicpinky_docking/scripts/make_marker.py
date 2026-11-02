#!/usr/bin/env python3
"""ArUco 마커를 실제 크기 PNG 로 만든다 (300 DPI).

A4 에 "페이지에 맞춤" 을 끄고 100% 로 인쇄하면 검은 사각형 한 변이 --size 와 같다.

    ros2 run vicpinky_docking make_marker.py --id 0 --size 80 -o aruco_0.png

Gazebo 텍스처처럼 여백·글씨 없이 마커만 필요하면 --margin 0 --no-label.
"""
import argparse

import cv2
import numpy as np
from PIL import Image

DPI = 300
MM_PER_INCH = 25.4

DICT_NAMES = {v: k for k, v in vars(cv2.aruco).items() if k.startswith('DICT_')}


def mm_to_px(mm):
    return int(round(mm / MM_PER_INCH * DPI))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--id', type=int, default=0, help='마커 번호 (기본 0)')
    p.add_argument('--size', type=float, default=80.0, help="마커 한 변 mm (기본 80)")
    p.add_argument('--dict', type=int, default=10,
                   help='cv2.aruco 사전 번호 (기본 10 = DICT_6X6_250)')
    p.add_argument('--margin', type=float, default=20.0, help='흰 여백 mm (기본 20)')
    p.add_argument('--no-label', action='store_true', help='아래 설명 글씨를 넣지 않는다')
    p.add_argument('-o', '--output', default=None, help='출력 파일 (기본 aruco_<id>_<size>mm.png)')
    args = p.parse_args()

    dict_name = DICT_NAMES.get(args.dict, f'DICT_{args.dict}')
    out = args.output or f'aruco_{args.id}_{int(args.size)}mm.png'

    dictionary = cv2.aruco.getPredefinedDictionary(args.dict)
    side = mm_to_px(args.size)
    if hasattr(cv2.aruco, 'generateImageMarker'):  # OpenCV >= 4.7
        marker = cv2.aruco.generateImageMarker(dictionary, args.id, side, borderBits=1)
    else:  # Ubuntu 24.04 기본 OpenCV 4.6
        marker = cv2.aruco.drawMarker(dictionary, args.id, side, borderBits=1)

    margin = mm_to_px(args.margin)
    label_h = 0 if args.no_label else mm_to_px(8)
    w = side + 2 * margin
    h = side + 2 * margin + label_h
    img = np.full((h, w), 255, np.uint8)
    img[margin:margin + side, margin:margin + side] = marker

    if not args.no_label:
        text = f'ID={args.id}  {dict_name}  {args.size:g}mm'
        scale = mm_to_px(3.5) / 22.0  # Hershey simplex 글자 높이 약 22px @ scale 1
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)
        org = ((w - tw) // 2, margin + side + margin // 2 + th)
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, 0, 2, cv2.LINE_AA)

    # DPI 를 파일에 적어 두어야 인쇄 프로그램이 실제 크기로 찍는다
    Image.fromarray(img).save(out, dpi=(DPI, DPI))
    print(f'{out}: ID={args.id} {dict_name}, 마커 {args.size:g} mm = {side}px @ {DPI} DPI, '
          f'전체 {w / DPI * MM_PER_INCH:.0f} x {h / DPI * MM_PER_INCH:.0f} mm')
    print('인쇄: "페이지에 맞춤" 끄고 100% 로. 인쇄 후 검은 사각형 한 변을 자로 재서 확인할 것.')


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""카메라 캘리브레이션용 체커보드를 실제 크기 PNG + PDF(A4) 로 만든다.

주의: camera_calibration 의 --size 는 *칸 수* 가 아니라 *내부 꼭짓점 수* 다.
      9x7 칸 보드 → --size 8x6

    ros2 run vicpinky_docking make_checkerboard.py --cols 9 --rows 7 --square 25
"""
import argparse

import numpy as np
from PIL import Image

DPI = 300
MM_PER_INCH = 25.4
A4_MM = (297.0, 210.0)  # 가로 방향


def mm_to_px(mm):
    return int(round(mm / MM_PER_INCH * DPI))


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--cols', type=int, default=9, help='가로 칸 수 (기본 9)')
    p.add_argument('--rows', type=int, default=7, help='세로 칸 수 (기본 7)')
    p.add_argument('--square', type=float, default=25.0, help='칸 한 변 mm (기본 25)')
    p.add_argument('-o', '--output', default=None, help='출력 파일 이름 앞부분 (확장자 제외)')
    args = p.parse_args()

    inner_c, inner_r = args.cols - 1, args.rows - 1
    stem = args.output or (f'checkerboard_{args.cols}x{args.rows}squares_'
                           f'{inner_c}x{inner_r}inner_{args.square:g}mm')

    sq = mm_to_px(args.square)
    board = np.zeros((args.rows * sq, args.cols * sq), np.uint8)
    for r in range(args.rows):
        for c in range(args.cols):
            if (r + c) % 2 == 1:
                board[r * sq:(r + 1) * sq, c * sq:(c + 1) * sq] = 255

    page_w, page_h = mm_to_px(A4_MM[0]), mm_to_px(A4_MM[1])
    if board.shape[1] > page_w or board.shape[0] > page_h:
        raise SystemExit(f'보드가 A4 가로({A4_MM[0]:g}x{A4_MM[1]:g} mm)보다 크다. '
                         f'--square 를 줄여라.')
    page = np.full((page_h, page_w), 255, np.uint8)
    y0, x0 = (page_h - board.shape[0]) // 2, (page_w - board.shape[1]) // 2
    page[y0:y0 + board.shape[0], x0:x0 + board.shape[1]] = board

    img = Image.fromarray(page)
    img.save(stem + '.png', dpi=(DPI, DPI))
    img.convert('RGB').save(stem + '.pdf', resolution=DPI)

    print(f'{stem}.png / .pdf  (A4 가로, {DPI} DPI)')
    print(f'  칸 수        : {args.cols} x {args.rows}')
    print(f'  내부 꼭짓점  : {inner_c} x {inner_r}   ← camera_calibration --size 에 넣는 값')
    print(f'  칸 한 변     : {args.square:g} mm  ← --square {args.square / 1000:g}')
    print(f'인쇄 후 칸 한 변을 자로 재서 {args.square:g} mm 인지 확인하고, 다르면 잰 값을 쓸 것.')
    print(f'예) ros2 run camera_calibration cameracalibrator --size {inner_c}x{inner_r} '
          f'--square {args.square / 1000:g} --ros-args -r image:=/camera/image_raw '
          f'-p camera:=/camera')


if __name__ == '__main__':
    main()

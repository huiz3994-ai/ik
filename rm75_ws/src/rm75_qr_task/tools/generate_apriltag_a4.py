#!/usr/bin/env python3
"""Generate four vector A4 AprilTag 36h11 station sheets at exact scale."""

import argparse
import os

from reportlab.lib.colors import black, HexColor
from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfbase.pdfmetrics import stringWidth
from reportlab.pdfgen.canvas import Canvas


# 8x8 modules including the one-module black border. These matrices were
# generated from OpenCV DICT_APRILTAG_36h11 with borderBits=1. 1 means black.
TAG_BITS = {
    0: (
        '11111111', '11101111', '11001011', '11110101',
        '11110011', '10100011', '10101001', '11111111'),
    1: (
        '11111111', '10110111', '10100101', '11110011',
        '11100001', '10001011', '11001001', '11111111'),
    2: (
        '11111111', '11000111', '11101111', '10110111',
        '11111101', '11011011', '10001001', '11111111'),
    3: (
        '11111111', '11110011', '11011001', '10110101',
        '10001101', '10001111', '10110001', '11111111'),
}


def centered_text(canvas, text, y, font='Helvetica', size=10, color=black):
    canvas.setFont(font, size)
    canvas.setFillColor(color)
    canvas.drawString((A4[0] - stringWidth(text, font, size)) / 2.0, y, text)


def draw_tag(canvas, tag_id, left, bottom, size):
    bits = TAG_BITS[tag_id]
    module = size / 8.0
    canvas.setFillColor(black)
    for row, values in enumerate(bits):
        for column, value in enumerate(values):
            if value == '1':
                canvas.rect(left + column * module,
                            bottom + (7 - row) * module,
                            module, module, stroke=0, fill=1)


def draw_scale(canvas, left, baseline):
    canvas.setStrokeColor(black)
    canvas.setFillColor(black)
    canvas.setLineWidth(0.35 * mm)
    canvas.line(left, baseline, left + 100 * mm, baseline)
    for index in range(11):
        x = left + index * 10 * mm
        tick = 5 * mm if index in (0, 5, 10) else 3 * mm
        canvas.line(x, baseline - tick / 2, x, baseline + tick / 2)
        if index in (0, 5, 10):
            label = str(index * 10)
            canvas.setFont('Helvetica', 8)
            canvas.drawCentredString(x, baseline - 8 * mm, label)
    canvas.setFont('Helvetica-Bold', 9)
    canvas.drawCentredString(left + 50 * mm, baseline + 5 * mm,
                             '100 mm verification scale')


def create_pdf(output_path, marker_size_mm=80.0):
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    canvas = Canvas(output_path, pagesize=A4, pageCompression=1)
    canvas.setTitle('RM75 AprilTag 36h11 stations 0-3, A4, 80 mm')
    canvas.setAuthor('RM75 project')
    tag_size = marker_size_mm * mm
    left = (A4[0] - tag_size) / 2.0
    bottom = 125 * mm

    for page_number, tag_id in enumerate(range(4), start=1):
        canvas.setFillColor(HexColor('#172033'))
        canvas.rect(0, A4[1] - 28 * mm, A4[0], 28 * mm, stroke=0, fill=1)
        centered_text(canvas, 'RM75 FOUR-STATION MARKER', A4[1] - 14 * mm,
                      'Helvetica-Bold', 16, HexColor('#FFFFFF'))
        centered_text(canvas, 'AprilTag 36h11  |  SLOT_{}  |  ID {}'.format(
            tag_id, tag_id), A4[1] - 22 * mm,
            'Helvetica', 10, HexColor('#D7E3FF'))

        centered_text(canvas, 'OUTER BLACK SQUARE = {:.1f} mm'.format(marker_size_mm),
                      bottom + tag_size + 13 * mm, 'Helvetica-Bold', 12)
        draw_tag(canvas, tag_id, left, bottom, tag_size)

        canvas.setStrokeColor(HexColor('#748096'))
        canvas.setLineWidth(0.25 * mm)
        canvas.line(left - 8 * mm, bottom, left - 8 * mm, bottom + tag_size)
        canvas.line(left - 10 * mm, bottom, left - 6 * mm, bottom)
        canvas.line(left - 10 * mm, bottom + tag_size,
                    left - 6 * mm, bottom + tag_size)
        canvas.saveState()
        canvas.translate(left - 12 * mm, bottom + tag_size / 2)
        canvas.rotate(90)
        canvas.setFont('Helvetica', 8)
        canvas.setFillColor(HexColor('#566176'))
        canvas.drawCentredString(0, 0, '{:.1f} mm'.format(marker_size_mm))
        canvas.restoreState()

        draw_scale(canvas, (A4[0] - 100 * mm) / 2.0, 78 * mm)
        centered_text(canvas, 'PRINT: Actual size / 100%  -  Disable Fit or Shrink',
                      52 * mm, 'Helvetica-Bold', 11, HexColor('#9B1C1C'))
        centered_text(canvas, 'After printing, measure the black square and 100 mm scale.',
                      44 * mm, 'Helvetica', 9)
        centered_text(canvas, 'If the black square is not 80.0 mm, enter the measured value in markers.yaml.',
                      38 * mm, 'Helvetica', 9)
        centered_text(canvas, 'Keep the white area around the tag unobstructed and mount the sheet flat.',
                      32 * mm, 'Helvetica', 9)

        canvas.setStrokeColor(HexColor('#D2D7E0'))
        canvas.line(18 * mm, 20 * mm, A4[0] - 18 * mm, 20 * mm)
        canvas.setFillColor(HexColor('#566176'))
        canvas.setFont('Helvetica', 8)
        canvas.drawString(18 * mm, 13 * mm,
                          'RM75_SLOT_{}  |  page {}/4'.format(tag_id, page_number))
        canvas.drawRightString(A4[0] - 18 * mm, 13 * mm,
                               'A4 210 x 297 mm')
        canvas.showPage()
    canvas.save()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output')
    parser.add_argument('--marker-size-mm', type=float, default=80.0)
    args = parser.parse_args()
    if args.marker_size_mm <= 0:
        raise SystemExit('marker size must be positive')
    create_pdf(args.output, args.marker_size_mm)


if __name__ == '__main__':
    main()

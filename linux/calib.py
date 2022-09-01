#!/usr/bin/python
from __future__ import print_function

import argparse
import yaml
import sys
from numpy import interp


def process_args(argv=sys.argv[1:]):
    parser = argparse.ArgumentParser()
    parser.add_argument('--calib', '-c', type=argparse.FileType('r'), metavar="CALIB_FILE",
                        required=True, help='YAML calibration file')
    parser.add_argument('file', type=argparse.FileType('r'), metavar="INPUT_FILE",
                        nargs='?', default=sys.stdin, help='data file to transform')
    return parser.parse_args(argv)


def load_calib(file):
    calib = yaml.load(args.calib)
    xs = sorted(calib.keys())
    ys = [calib[k] for k in xs]
    return xs, ys


args = process_args()
xs, ys = load_calib(args.calib)

out = sys.stdout
for line in args.file.readlines():
    elems = line.split(';')
    try:
        assert len(elems) > 1
        result = [elems[0]]
        calibrated = [str(interp(float(x), xs, ys)) for x in elems[1:] if not x.isspace()]
        result.extend(calibrated)
        print(';'.join(result), file=out)
    except (AssertionError, ValueError):
        out.write(line)

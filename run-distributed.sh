#!/bin/sh
erl -rsh ssh -sname master -noinput -run raytracer standalone 640 480 "/tmp/traced.ppm" 1 distributed

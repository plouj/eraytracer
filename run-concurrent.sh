#!/bin/sh
erl -noinput -run raytracer standalone 16 12 "/tmp/traced.ppm" 1 concurrent

#!/bin/sh
erl -noinput -run raytracer standalone 32 24 "/tmp/traced.ppm" 1 simple

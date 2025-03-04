#!/bin/bash
doc/prebuild.sh; sphinx-build doc _build; rm -r doc/bob/src

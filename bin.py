import os
import csv
import base64
import sys
import traceback
import numpy as np
import cv2
import gzip
from datetime import datetime
from flask import Flask, request, jsonify, render_template, Response
from flask_cors import CORS
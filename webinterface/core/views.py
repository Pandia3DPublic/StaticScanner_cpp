import datetime
import time
from core import socketio
from flask import Blueprint, render_template, request, flash, redirect, url_for, jsonify, send_file, abort, \
    send_from_directory,Response
from flask_login import login_user, login_required, current_user, logout_user
from . import db
from datetime import datetime, timedelta
import json
import random,requests
from random import randrange
import time
import threading

views = Blueprint('views', __name__)
clients = []

@views.route('/')
@login_required
def home():
    # socketio.emit('neuePunktwolke', {'datensatz': randrange(10)}, broacast=True)
    return render_template("pandia.html", user=current_user)



@socketio.on("testcall")
def testfunction(data):
    print(data)


@socketio.on("car_position")
def car_position(data):
    socketio.emit("updateCarPosition", data, broadcast=True)





import random
from flask import Flask
from flask_sqlalchemy import SQLAlchemy
from os import path
from flask_login import LoginManager
from flask_restful import Api, Resource, reqparse, abort, fields, marshal_with
from flask_sqlalchemy import SQLAlchemy
from werkzeug.security import generate_password_hash
from flask_socketio import SocketIO
from sys import platform

from datetime import timedelta, datetime

if "linux" in platform:
    DB_NAME = 'database.db'
else:
    DB_NAME = 'database.db'

app = Flask(__name__) #name is folder name
app.config['SECRET_KEY'] = 'awdhiauohduiawhduiaud'
app.config['SQLALCHEMY_DATABASE_URI'] = f'sqlite:///{DB_NAME}'
db = SQLAlchemy(app)
api = Api(app)
socketio = SocketIO(app)


from .views import views
from .auth import auth
from .models import *
from .fillup_database import create_database
from .api_base import *

login_manager = LoginManager()
login_manager.login_view = 'auth.login'
login_manager.init_app(app)

create_database(app, db)

@login_manager.user_loader
def load_user(user_id):
    return User.query.get(int(user_id))



app.register_blueprint(views, url_prefix='/')
app.register_blueprint(auth, url_prefix='/')



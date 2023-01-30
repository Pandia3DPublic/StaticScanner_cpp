from werkzeug.security import generate_password_hash

from .models import User
import datetime
import random


def create_database(app, db_i):
    db_i.create_all(app=app)

    try:
        ##Random Users
        admin = User(email="admin@thw.de", first_name="Adi",
                     password=generate_password_hash("test123", method='sha256'))

        try:
            db_i.session.add(admin)
            db_i.session.commit()
        except:
            print("username existed")
        print("Created Database Successfully!")
    except:
        print("Database already existed")


if __name__ == "__main__":
    create_database()

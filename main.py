from Controller import Controller


def main():
    try:
        c = Controller("COM3")
        c.connect()
        c.getData()
    except Exception as e:
        print(e.__doc__)
        c.CleanUpPort()


if __name__ == "__main__":
    c = main()

from myThymio import MyThymio

TIME_STEP = 0.1


def main():
    # myThymio = MyThymio(port="/dev/cu.usbmodem14401", robot_path=robot_path, refreshing_rate=2 * TIME_STEP)
    myThymio = MyThymio(port="COM5", refreshing_rate=2 * TIME_STEP, initial_pos=(200, 300, 0))
    # myThymio = MyThymio(port="COM3", refreshing_rate=2 * TIME_STEP, initial_pos=(444, 147, 0), save_data=True)
    myThymio.run()


if __name__ == "__main__":
    main()

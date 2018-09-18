import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import time

def main():
    # data for plotting
    t = np.arange(0.0, 2.0, 0.01)
    s = 1 + np.sin(2*np.pi*t)

    # set up plot
    plt.axis([-0.1, 2.1, 0, 2.1])
    plt.ion()
    plt.show()

    # plot data
    plt.plot(t,s)
    plt.draw()
    plt.pause(0.001)

    raw_input("Press [enter] to continue.")

    print("Test")

if __name__ == '__main__':
    main()

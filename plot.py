import matplotlib.pyplot as plt


def main():
    out = []
    # f = open("vel.log")
    f = open("plots/vel.log")
    
    measurement = []
    reference = []
    error = []
    p_term = []
    i_term = []
    d_term = []

    for r in f:
        try:
            vals = [float(s) for s in vals.split() if s.isdigit()]
            if len(vals == 5):
                measurement.apppend(vals[0])
                error.apppend(vals[1])
                p_term.apppend(vals[2])
                i_term.apppend(vals[3])
                d_term.apppend(vals[4])
                reference.apppend(vals[5])

        except ValueError:
            pass

    print (len(measurement))

    plt.figure()
    plt.plot(measurement, label="measurement")
    plt.plot(reference, label="reference")
    plt.grid()
    plt.legend()

    plt.figure()
    plt.plot(error, label="error")
    plt.plot(p_term, label="proportional")
    plt.plot(i_term, label="integral")
    plt.plot(d_term, label="derivative")
    plt.grid()
    plt.legend()
    plt.show()

if (__name__ == "__main__"):
    main()


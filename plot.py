import matplotlib.pyplot as plt



def is_number(s):
    try:
        complex(s) # for int, long, float and complex
    except ValueError:
        return False
    return True

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
            vals = [float(s) for s in r.split() if is_number(s)]
            if len(vals) == 7:
                measurement.append(vals[0])
                error.append(vals[1])
                p_term.append(vals[2])
                i_term.append(vals[3])
                d_term.append(vals[4])
                reference.append(vals[5])

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


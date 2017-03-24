import matplotlib.pyplot as plt

REF = 17

def main():
    out = []
    f = open("plots/vel.log")
    
    for r in f:
        try:
            val = float(r)
            out.append(val)
        except ValueError:
            pass

    ref = [REF]*len(out)

    print len(out)

    plt.plot(out, label="output")
    plt.plot(ref, label="reference")
    plt.grid()
    plt.legend()
    plt.show()

if (__name__ == "__main__"):
    main()


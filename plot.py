import matplotlib.pyplot as plt

main():
    out = []
    f = open("plots/vel.log")
    
    for r in f:
        val = float(r)
        out.append(val)        

    ref = [REF]*len(out)

    plt.plot(out, label="output")
    plt.plot(ref, label="reference")
    plt.grid()
    plt.legend()
    plt.show()

if (__name__ == "__main__"):
    main()


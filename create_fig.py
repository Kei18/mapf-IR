import os
import re
import glob
import matplotlib.pyplot as plt
import datetime


def read_data(filename):
    date = ""
    results = []
    with open(filename) as f:
        mode = 0  # sub-optimal -> optimal -> anytime
        r_date = re.compile(r"date:\s(.+)")
        r_pattern = re.compile(r"\|\s(.+)\s\|\s\d\s\|\s(\d+)\s\|\s\d+\s\|\s\d+\s\|")
        r_opt = re.compile(r"\#\#\soptimal\ssolvers")
        r_any = re.compile(r"\#\#\sanytime\ssolvers")

        for row in f:
            m_opt = re.match(r_opt, row)
            if m_opt:
                mode += 1
                continue
            m_any = re.match(r_any, row)
            if m_any: break

            m_date = re.match(r_date, row)
            if m_date:
                date = datetime.datetime.strptime(str(m_date.group(1)), '%Y-%m-%d-%H-%M-%S')
                continue

            m_pattern = re.match(r_pattern, row)
            if m_pattern:
                solver_name = str(m_pattern.group(1))
                comp_time = int(m_pattern.group(2))
                results.append((mode, solver_name, comp_time))
    return (date, results)

def argsort(arr):
    arr2 = list(range(len(arr)))
    arr2.sort(key=lambda x:arr[x])
    return arr2

if __name__ == '__main__':
    # read data
    all_data = []
    for filename in glob.glob(os.path.join("records", "*.md")):
        all_data.append(read_data(filename))
    formatted_data = { 0: {}, 1: {} }
    for date, results in all_data:
        for s, solver, comp_time in results:
            if solver not in formatted_data[s].keys():
                formatted_data[s][solver] = [ [], [] ]
            formatted_data[s][solver][0].append(date)
            formatted_data[s][solver][1].append(comp_time)

    # plot figures
    fontsize=16
    s=0
    for s in [0, 1]:
        plt.figure(figsize=(12,4))
        solvers = list(formatted_data[s].keys())
        for solver in solvers:
            arr = formatted_data[s][solver][0]
            index = argsort(arr)
            y_raw_data = [formatted_data[s][solver][1][i] for i in index]
            x_raw_data = [formatted_data[s][solver][0][i] for i in index]
            y_data = [ y_raw_data[0]/1000 ]
            x_data = [ x_raw_data[0] ]
            for i in range(1, len(y_raw_data)):
                y_data.append(y_raw_data[i-1]/1000)
                y_data.append(y_raw_data[i]/1000)
                x_data.append(x_raw_data[i])
                x_data.append(x_raw_data[i])
            plt.plot(x_data, y_data, label=solver, alpha=0.9)
            plt.scatter([x_data[0], x_data[-1]], [y_data[0], y_data[-1]], marker='x', s=200)

        plt.gca().spines['right'].set_visible(False)
        plt.gca().spines['top'].set_visible(False)
        plt.title("sub-optimal solvers, den520d, 300agents" if s == 0 else "optimal solvers, random-32-32-20, 30agents", fontsize=fontsize)
        plt.xticks(rotation=45, fontsize=fontsize-2)
        plt.yticks(fontsize=fontsize-2)
        plt.yscale('log')
        plt.ylabel("runtime (sec)")
        plt.legend(frameon=False, fontsize=fontsize-2, bbox_to_anchor=(1.0,1.0))
        plt.savefig("./fig/transition_{}.pdf".format(s), pad_inches=0.05, transparent=False, bbox_inches='tight')

import re
import datetime
import subprocess

optimal_solvers = [
    'CBS',
    'ICBS',
]

suboptimal_solvers = [
    'PIBT',
    'HCA',
    'WHCA',
    'RevisitPP',
    'ECBS',
    'PIBT_COMPLETE',
]

output_file = './result.txt'
cmd_app = '../build/app'
cmd_output = '-o ' + output_file
optimal_ins = './benchrmark/optimal_random-32-32-20_30agents.txt'
suboptimal_ins = './benchrmark/suboptimal_arena_300agents.txt'

def read_data(filename):
    r_solved = re.compile(r"solved=(\d)")
    r_makespan = re.compile(r"makespan=(\d+)")
    r_soc = re.compile(r"soc=(\d+)")
    r_comp_time = re.compile(r"comp_time=(\d+)")
    result = {}
    with open(output_file, 'r') as f:
        for row in f:
            m_solved = re.match(r_solved, row)
            if m_solved:
                result['solved'] = int(m_solved.group(1))
                continue
            m_comp_time = re.match(r_comp_time, row)
            if m_comp_time:
                result['comp_time'] = int(m_comp_time.group(1))
                continue
            m_soc = re.match(r_soc, row)
            if m_soc:
                result['sum-of-costs'] = int(m_soc.group(1))
                continue
            m_makespan = re.match(r_makespan, row)
            if m_makespan:
                result['makespan'] = int(m_makespan.group(1))
                continue
    return result

def run(ins, solver):
    cmd_ins = '-i ' + ins
    cmd_solver = '-s ' + solver
    cmd = '{} {} {} {}'.format(cmd_app, cmd_output, cmd_ins, cmd_solver)
    print(cmd)
    subprocess.run(cmd, shell=True, encoding='utf-8', stdout=subprocess.PIPE)
    return read_data(output_file)

def get_run_result(ins, solvers):
    table_header = """|solver | solved | comp_time(ms) | sum-of-costs | makespan |
| --- | --- | ---: | ---: | ---: |
"""
    result_str = table_header
    for solver in solvers:
        res = run(ins, solver)
        result_str += "| {} | {} | {} | {} | {} |\n".format(
            solver, res['solved'], res['comp_time'], res['sum-of-costs'], res['makespan'])
    return result_str


if __name__ == '__main__':
    date_str = datetime.datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
    cp = subprocess.run('git --no-pager log -n 1 --no-decorate',
                        shell=True, encoding='utf-8', stdout=subprocess.PIPE)
    git_log = cp.stdout

    suboptimal_res = get_run_result(suboptimal_ins, suboptimal_solvers)
    optimal_res = get_run_result(optimal_ins, optimal_solvers)

    md_str = """auto record by github actions
===
date: {}

commit
```
{}
```

## sub-optimal solvers
benchmark: {}

{}
## optimal solvers
benchmark: {}

{}""".format(
    date_str,
    git_log,
    suboptimal_ins,
    suboptimal_res,
    optimal_ins,
    optimal_res)

    record_file = './readme.md'
    with open(record_file, 'w') as f:
        f.write(md_str)

import re
import subprocess

filename = 'docs/index.html'
r_last_date = re.compile(r".+id=\"last-updated\".+")
new_file = ""

cp = subprocess.run('git --no-pager log --pretty=format:"%ad" --date=short',
                    shell=True, encoding='utf-8', stdout=subprocess.PIPE)
hist = cp.stdout
hists = hist.split('\n')
updated_date = hists[0]

with open(filename) as f:
    for row in f:
        m_last_date = re.match(r_last_date, row)
        if m_last_date:
            new_file += '<span id="last-updated">(last updated: {})</span>'.format(updated_date)
        else:
            new_file += row

with open(filename, 'w') as f:
    f.write(new_file)

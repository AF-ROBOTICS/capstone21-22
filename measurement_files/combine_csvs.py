import pandas as pd
import sys
import os
import glob
from pathlib import Path

extension = 'csv'
all_filenames = [i for i in glob.glob('*.{}'.format(extension))]

writer = pd.ExcelWriter('TestData.xlsx')  # Output name
for filename in all_filenames:

    print("Loading " + filename)
    df = pd.read_csv(filename)
    df.set_index('bot', inplace=True)
    df.to_excel(writer, sheet_name=os.path.splitext(filename)[0])
    print("done")
writer.save()
print("task completed")
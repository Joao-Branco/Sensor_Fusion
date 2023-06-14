import math
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import numpy as np


df1 = pd.DataFrame({'Time': range(1,20,1), 'x': range(10, 29,1), 'y': range(10, 29,1)})

df2 = pd.DataFrame({'Time': [i*0.3 for i in range(50)], 'x': range(50), 'y': range(50)})

df1_t = df1.set_index('Time')

print(df1)

print(df2)

df2_t = df2.set_index('Time')

real_index = df1_t.index

df_m = df1_t.join(df2_t[['x', 'y']].add_suffix('2'), how='outer')


print(df_m)

# as we can see, we applied the filter to the original index from df1_t, losing values from df2_t
# but we want to interpolate first

df_f = df_m.interpolate().loc[real_index]
print(df_f)
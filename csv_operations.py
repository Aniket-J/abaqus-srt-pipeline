import csv
import pandas as pd

df = pd.read_csv(r'C:\Users\acj249\Work Folders\Desktop\Abaqus_explicit\SRT_Abaqusrp_output.csv',sep=',',names=["Time", "U2", "Timerf2", "RF2"])

df = df.drop(['Timerf2'], axis = 1)
df = df[["Time", "RF2", "U2"]]

print(df.head())
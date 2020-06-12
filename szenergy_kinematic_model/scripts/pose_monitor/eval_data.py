'''
Created on Jun 12, 2020

@author: kyberszittya
'''

import pandas

import argparse
import matplotlib.pyplot as plt


parser = argparse.ArgumentParser(description='Data reader and analyzer')

x = pandas.read_csv("./data/eval_no_odom.csv", sep=';')
x_odom = pandas.read_csv("./data/eval_with_odom.csv", sep=';')
plt.figure()
plt.hist(x["yaw_error"])
plt.hist(x_odom["yaw_error"])
plt.figure()
plt.hist(x["ref_error"])
plt.hist(x_odom["ref_error"])
plt.figure()
plt.hist(x["euclidean_error"])
plt.hist(x_odom["euclidean_error"], alpha=0.7)
plt.legend()
print("euc: {0}: {1}".format(x["euclidean_error"].max(), x_odom["euclidean_error"].max()))
print("ref: {0}: {1}".format(x["ref_error"].max(), x_odom["ref_error"].max()))
print("yaw: {0}: {1}".format(x["yaw_error"].max(), x_odom["yaw_error"].max()))
plt.show()
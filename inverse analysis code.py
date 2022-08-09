import numpy as np
import scipy.optimize as optimize


iternum = 'Test_0-552'
forcename = 'Force N '
sdispname = 'Punch Displacement mm '

itertext = 'c:/temp/' + iternum + '.rpt'


def modelfunc(x):
    simfile = np.loadtxt(itertext, delimiter='\t')
    simdata = np.array(simfile)
    simy1 = simdata[:, 1]  # force
    simy = simy1 / 1000
    simx = simdata[:, 2]  # displacement
    simdisp = np.linspace(0.06, 2, 100)
    simint = np.interp(simdisp, simx, simy)
    sim_data_array = np.array(simint)
    iteration = iteration + 1
    return sim_data_array


def obj(x, *ex_data_array):
    print('x = ', x)
    JCcalcs = modelfunc(x)
    print('simulated data array:', JCcalcs)
    r = np.sum((JCcalcs - ex_data_array) ** 2)
    print('r = ', r)
    return r



exfile = np.loadtxt('C:/temp/experimental_Data.txt', skiprows=1)
exdata = np.array(exfile)
exx = exdata[:, 1]  # displacement
exy1 = exdata[:, 0]  # force
exy = exy1/1000
exdisp = np.linspace(0.06, 2, 100)
exint = np.interp(exdisp, exx, exy)
ex_data_array = np.array(exint)
x0 = np.array([641.11, 666.4, 0.459, 1.615])
res = optimize.minimize(obj, x0, method='Nelder-Mead', args=ex_data_array)
print('res = ', res['x'])

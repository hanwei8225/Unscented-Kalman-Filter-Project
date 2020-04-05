import sys
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

matplotlib.rc('text', usetex=True)
matplotlib.rcParams['text.latex.preamble'] = [r'\boldmath']
matplotlib.rcParams['axes.linewidth'] = 2

matplotlib.rcParams['axes.labelsize'] = 20

matplotlib.rcParams['lines.linewidth'] = 2
matplotlib.rcParams['lines.markersize'] = 6

matplotlib.rcParams['xtick.major.size'] = 6
matplotlib.rcParams['xtick.minor.size'] = 4
matplotlib.rcParams['xtick.major.width'] = 2
matplotlib.rcParams['xtick.minor.width'] = 2
matplotlib.rcParams['xtick.labelsize'] = 20
matplotlib.rcParams['xtick.minor.pad'] = 10
matplotlib.rcParams['xtick.major.pad'] = 10

matplotlib.rcParams['ytick.major.size'] = 6
matplotlib.rcParams['ytick.minor.size'] = 4
matplotlib.rcParams['ytick.major.width'] = 2
matplotlib.rcParams['ytick.minor.width'] = 2
matplotlib.rcParams['ytick.labelsize'] = 20

# Normalized innovation squared (NIS) data values for 
# radar and laser measurements
dataRadar = np.loadtxt("./NIS_radar.txt", usecols=[0], skiprows=1)
dataLaser = np.loadtxt("./NIS_laser.txt", usecols=[0], skiprows=1)
# We skip the first row to cut out the unrealistically high NIS value 
# from the first measurement.  The Kalman filter has not found its groove yet.

nisRadar = np.transpose(dataRadar)
nisLaser = np.transpose(dataLaser)
nis = [nisRadar, nisLaser]

# Laser measurements have 3 degrees of freedom.
# confidence95Laser is the threshold at which the p-value of the 
# NIS distribution is 0.05.
# Put another way, the probability that the NIS value 
# is >= confidence95Laser should be 0.05, if the process
# noise values are consistent.
# Put a third way, when we plot the NIS values for the
# set of radar measurements, roughly 5% of them should be
# above the 7.82 threshold, if the noise values are consistent.
# 
# This serves as a check on our choice of process noise values.
confidence95Radar = 7.82

# Laser measurements have 2 degrees of freedom,
# so the threshold is different.
confidence95Laser = 5.99

confidences = np.array([confidence95Radar, confidence95Laser])

fig = plt.figure(figsize=(16,8))
axes = []
# ax.tick_params(which='both',direction='in')
# ax.set_xlabel( r'$\bar{NIS}$' )
id = 1
for data, confidence in zip(nis, confidences):
    subplot = 120 + id
    ax = fig.add_subplot(subplot)
    axes.append( ax )
    ax.tick_params(which='both',direction='in')
    ax.set_xlabel( r'$\textrm{\textbf{Measurement index}}$', labelpad=10 )
    ax.plot( np.arange(0,len(data))
        , data
        , 'r-'
        , label=r'$\textrm{\textbf{NIS}}$' )
    ax.axhline( y=confidence
        , color='b'
        , linestyle='-'
        , label=r'$\textrm{\textbf{95\% confidence threshold}}$' )
    ax.legend( prop={'size':20} )
    id += 1

# fig.suptitle(r'$\textrm{\textbf{NIS values with 95\% confidence interval}}$',
#              fontsize = 20)
axes[0].set_title(r'$\textrm{\textbf{Radar}}$', fontsize=20)
axes[1].set_title(r'$\textrm{\textbf{Laser}}$', fontsize=20)

# plt.plot( nbarvalsCont
#     , vecFRH( nbarvalsCont )
#     , 'b-'
#     , label=r'$\textrm{\textbf{95\% confidence threshold}}$' )

plt.tight_layout()
plt.savefig( "NIS.png", bbox_inches = 'tight', dpi = 300 )

plt.show()

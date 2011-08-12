#! /usr/bin/python

import pylab
import matplotlib.pyplot as plt
import numpy as np
from optparse import OptionParser
import sys

if __name__ == "__main__":
    
    parser = OptionParser()
    parser.add_option("-i", "--input_file", dest="input_file", 
                      help="input FILE", 
                      metavar="FILE", action="store")
    
    (options, _) = parser.parse_args()
    
    if options.input_file is None:
        print "The input file has to be specified"
        sys.exit()
    
    scale = 3.0 
    text_size = 10.0 * scale
    tick_size = 6.0 * scale
    
    fig_width_pt = 252.0  # Get this from LaTeX using \showthe\columnwidth
    inches_per_pt = 1.0/72.27               # Convert pt to inch
    golden_mean = (np.sqrt(5.0)-1.0)/2.0         # Aesthetic ratio
    fig_width = scale * fig_width_pt*inches_per_pt  # width in inches
    fig_height = fig_width*golden_mean      # height in inches
    fig_size =  [fig_width ,fig_height ]
    params = {'backend': 'QT',
              'axes.labelsize': 0.8*text_size,
              'text.fontsize': text_size,
              'axes.titlesize': 0.8*text_size,
              'legend.fontsize': text_size,
              'xtick.labelsize': tick_size,
              'ytick.labelsize': tick_size,
              'text.usetex': True,
              'figure.figsize': fig_size,
              'font.family' : 'serif',
              'font.serif' :  'Times'}
    pylab.rcParams.update(params)
    
    data = np.load(options.input_file)
    
    prob = data['prob']
    means = data['means']
    stds = data['stds']
    
    fig = plt.figure(1);
    ax = fig.add_subplot(111, autoscale_on=True)

    stds[-6:] -= 10
    stds[stds<0] = 0
    
    ax.errorbar(1.0 - prob, means, yerr=stds, fmt='o')
    ax.set_xlabel(r'failure probability')
    ax.set_ylabel(r'$\bar{f}$', rotation='horizontal')
    ax.set_title("Average Fitness")
    ax.set_xlim(1.03, -0.03)
    #ax.set_xticks(np.arange(1.0, -0.05, -0.05))
    
    pylab.show()
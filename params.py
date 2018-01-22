"""
IZHI

NetPyNE wrappers for the different celltypes of Izhikevich neuron. 

Equations and parameter values taken from
  Izhikevich EM (2007).
  "Dynamical systems in neuroscience"
  MIT Press

Equation for synaptic inputs taken from
  Izhikevich EM, Edelman GM (2008).
  "Large-scale model of mammalian thalamocortical systems." 
  PNAS 105(9) 3593-3598.

Cell types available are based on Izhikevich, 2007 book:
    1. RS - Layer 5 regular spiking pyramidal cell (fig 8.12 from 2007 book)
    2. IB - Layer 5 intrinsically bursting cell (fig 8.19 from 2007 book)
    3. CH - Cat primary visual cortex chattering cell (fig8.23 from 2007 book)
    4. LTS - Rat barrel cortex Low-threshold  spiking interneuron (fig8.25 from 2007 book)
    5. FS - Rat visual cortex layer 5 fast-spiking interneuron (fig8.27 from 2007 book)
    6. TC - Cat dorsal LGN thalamocortical (TC) cell (fig8.31 from 2007 book)
    7. RTN - Rat reticular thalamic nucleus (RTN) cell  (fig8.32 from 2007 book)


Usage example:
    from neuron import h
    from params import TODO
    dummy = h.Section()
    cell = TODO(dummy)

Version: 2018jan20 by pablo
"""
from netpyne import specs

netParams = specs.NetParams() # object of class to store the network parameters

netParams.popParams['PMd'] = {'cellType':'', 'numCells':96, 'celModel':'Izhi'}
netParams.popParams['ASC'] = {'cellType':'', 'numCells':64, 'celModel':'Izhi'}
netParams.popParams['EDSC'] = {'cellType':'', 'numCells':64, 'celModel':'Izhi'}
netParams.popParams['IDSC'] = {'cellType':'', 'numCells':64, 'celModel':'Izhi'}
netParams.popParams['ER2'] = {'cellType':'', 'numCells':150, 'celModel':'Izhi'}
netParams.popParams['IF2'] = {'cellType':'', 'numCells':25, 'celModel':'Izhi'}
netParams.popParams['IL2'] = {'cellType':'', 'numCells':25, 'celModel':'Izhi'}
netParams.popParams['ER5'] = {'cellType':'', 'numCells':168, 'celModel':'Izhi'}
netParams.popParams['EB5'] = {'cellType':'', 'numCells':72, 'celModel':'Izhi'}
netParams.popParams['IF5'] = {'cellType':'', 'numCells':40, 'celModel':'Izhi'}
netParams.popParams['IL5'] = {'cellType':'', 'numCells':40, 'celModel':'Izhi'}
netParams.popParams['ER6'] = {'cellType':'', 'numCells':192, 'celModel':'Izhi'}
netParams.popParams['IF6'] = {'cellType':'', 'numCells':32, 'celModel':'Izhi'}
netParams.popParams['IL6'] = {'cellType':'', 'numCells':32, 'celModel':'Izhi'}


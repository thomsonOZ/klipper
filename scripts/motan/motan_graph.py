#!/usr/bin/env python
# Script to perform motion analysis and graphing
#
# Copyright (C) 2019-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import optparse, ast
import matplotlib
import readlog, analyzers


######################################################################
# Graphing
######################################################################

def plot_motion(amanager, graphs):
    # Generate data
    for graph in graphs:
        for dataset, plot_params in graph:
            amanager.setup_dataset(dataset)
    amanager.generate_datasets()
    datasets = amanager.get_datasets()
    # Build plot
    times = None
    fontP = matplotlib.font_manager.FontProperties()
    fontP.set_size('x-small')
    fig, rows = matplotlib.pyplot.subplots(nrows=len(graphs), sharex=True)
    if len(graphs) == 1:
        rows = [rows]
    rows[0].set_title("Motion Analysis")
    for graph, ax in zip(graphs, rows):
        for dataset, plot_params in graph:
            if times is None:
                seg_time = amanager.get_segment_time()
                times = [seg_time * i for i in range(len(datasets[dataset]))]
            label = amanager.get_label(dataset)
            ax.set_ylabel(label['units'])
            pparams = {'label': label['label'], 'alpha': 0.8}
            pparams.update(plot_params)
            ax.plot(times, datasets[dataset], **pparams)
            ax.legend(loc='best', prop=fontP)
            ax.grid(True)
    rows[-1].set_xlabel('Time (s)')
    return fig


######################################################################
# Startup
######################################################################

def setup_matplotlib(output_to_file):
    global matplotlib
    if output_to_file:
        matplotlib.use('Agg')
    import matplotlib.pyplot, matplotlib.dates, matplotlib.font_manager
    import matplotlib.ticker

def main():
    # Parse command-line arguments
    usage = "%prog [options] <logname>"
    opts = optparse.OptionParser(usage)
    opts.add_option("-o", "--output", type="string", dest="output",
                    default=None, help="filename of output graph")
    opts.add_option("-s", "--skip", type="float", default=0.,
                    help="Set the start time to graph")
    opts.add_option("-d", "--duration", type="float", default=5.,
                    help="Number of seconds to graph")
    opts.add_option("--segment-time", type="float", default=0.000100,
                    help="Analysis segment time (default 0.000100 seconds)")
    opts.add_option("-g", "--graph", help="Graph to generate (python literal)")
    options, args = opts.parse_args()
    if len(args) != 1:
        opts.error("Incorrect number of arguments")
    log_prefix = args[0]

    # Open data files
    lmanager = readlog.LogManager(log_prefix)
    lmanager.setup_index()
    lmanager.seek_time(options.skip)
    amanager = analyzers.AnalyzerManager(lmanager, options.segment_time)
    amanager.set_duration(options.duration)

    # Default graphs to draw
    graphs = [
        [('trapq:toolhead:velocity', {'color': 'green'})],
        [('trapq:toolhead:accel', {'color': 'green'})],
        [('deviation:stepq:stepper_x-kin:stepper_x', {'color': 'blue'})]
    ]
    if options.graph is not None:
        graphs = ast.literal_eval(options.graph)

    # Draw graph
    setup_matplotlib(options.output is not None)
    fig = plot_motion(amanager, graphs)

    # Show graph
    if options.output is None:
        matplotlib.pyplot.show()
    else:
        fig.set_size_inches(8, 6)
        fig.savefig(options.output)

if __name__ == '__main__':
    main()

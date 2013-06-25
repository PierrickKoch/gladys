#!/usr/bin/env python
"""
dot to json
===========

Convert from graphviz dot file to json in order to visualize a graph using d3.js
You must install pygraphviz and networkx, on Ubuntu:

    sudo apt-get install python-pygraphviz python-networkx

Then use the following template replacing the resulting 'force.json'
https://github.com/networkx/networkx/tree/master/examples/javascript/force
"""

def dot_to_json(file_in, file_out, indent=1):
    import json
    try:
        import networkx
        import pygraphviz
        from networkx.readwrite import json_graph
    except ImportError:
        print("Install pygraphviz and networkx:")
        print("sudo apt-get install python-pygraphviz python-networkx")
        return 1

    graph_dot  = pygraphviz.AGraph( file_in )
    graph_netx = networkx.from_agraph( graph_dot )
    graph_json = json_graph.node_link_data( graph_netx )

    # fix formatting [graphviz to d3.js]
    for node in graph_json["nodes"]:
        # replace label by name
        node['name'] = node.pop('label')
        # id from string to integer
        node['id']   = int(node['id'])

    with open(file_out, 'w') as f:
        json.dump(graph_json, f, indent=indent)

    return 0

def main(argv):
    if len(argv) < 3:
        print("usage: %s file_in.dot file_out.json"%argv[0])
        return 1

    return dot_to_json(argv[1], argv[2])

if __name__ == '__main__':
    import sys
    sys.exit(main(sys.argv))

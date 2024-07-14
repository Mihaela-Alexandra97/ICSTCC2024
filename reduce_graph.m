function [Adj_red,Edge_to_path,ind_red_to_full] = reduce_graph(Adj_full,ind_red)
%Reduce graph for applying TSP
% the new graph will be a mesh one, with # of nodes = length(ind_red)
% nodes with indices in "ind_red" are kept from Adj

%Output arguments:
%in reduced graph, node i corresponds to node with index "ind_red_to_full(i)" from full
%Adj_red in the adjacency matrix, with indices from reduced graph (from 1 to length(ind_red))
%Edge_to_path is a cell array, where for i,j nodes in the reduced graph,
% Edge_to_path{i,j} gives the path in the full graph from node "ind_red_to_full(i)" to "ind_red_to_full(j)"

ind_red = unique(ind_red);  %if duplicates, or not sorted

Adj_red = zeros(length(ind_red));
Edge_to_path = cell(length(ind_red));
ind_red_to_full = ind_red;

G_full = graph(Adj_full);    %full graph

pairs = nchoosek(ind_red,2);    %pairs to be connected, on rows
for i=1:size(pairs,1)
    ind_red_start = find(ind_red_to_full==pairs(i,1));
    ind_red_stop = find(ind_red_to_full==pairs(i,2));
    [path,cost] = shortestpath(G_full,pairs(i,1),pairs(i,2));

    Adj_red(ind_red_start,ind_red_stop) = cost;
    Adj_red(ind_red_stop,ind_red_start) = cost; %symmetric adjacency matrix

    Edge_to_path{ind_red_start,ind_red_stop} = path;
    Edge_to_path{ind_red_stop,ind_red_start} = flip(path);  %inverse order to traverse same path
end

end
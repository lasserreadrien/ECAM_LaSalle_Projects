## Author: adril <adril@LAPTOP-EJ1AIJHT>
## Created: 2023-01-08

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%function planPathPRM(mapFilePath, startPoint, endPoint, L1, L2)
%
% Task: Establish the path from the starting point to the goal, using the points defined with the PRM method
%       and using the Dijkstra method.
%
% Inputs:
%	- mapFilePath : the path to get the .mat file
% - startPoint : our starting point for the path planning
% - endPoint : our final goal
%
% Outputs:
%	- None
%
% Adrien Lasserre (adrien.lasserre@ecam.fr) &
% Gwenn Durpoix-Espinasson (g.durpoix-espinasson@ecam.fr)
% 08/01/2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function planPathPRM (mapFilePath, startPoint, endPoint, L1, L2)

  mapFile = load(mapFilePath);

  S = startPoint; %define the start as S
  G = endPoint;   %define the goal as G

  n = columns(mapFile.MatrixOfLinks); %get the size of the MatrixOfLinks
  connectionMatrix = zeros(n+2,n+2);  %initialize a new connection matrix, with size n+2 to enter the S and G
  connectionMatrix(2:n+1,2:n+1) = mapFile.MatrixOfLinks;%put the MatrixOfLinks inside

  points2Dmap = mapFile.Points'; %get the Points, but transposed (to use the findClosestPoint function)

  indexClosestPointS = findClosestPoint(S,points2Dmap) + 1  %+1 as in the first indexes of the connectionMatrix we put the S
  indexClosestPointG = findClosestPoint(G,points2Dmap) + 1  %display those results

  connectionMatrix(1, indexClosestPointS) = 1;              %use the previous results to update the connectionMatrix
  connectionMatrix(indexClosestPointS, 1) = 1;
  connectionMatrix(1, 1) = 1;
  connectionMatrix(n+2, indexClosestPointG) = 1;
  connectionMatrix(indexClosestPointG, n+2) = 1;
  connectionMatrix(n+2, n+2) = 1;

  points2D = [S; points2Dmap; G]                            %store the points (with S and G)
  connectionMatrix                                          %debugging

  [numberOfNodes, visibilityGraph] = createVisibilityGraph(connectionMatrix, points2D) %create the visibilityGraph


  [distanceToNode, parentOfNode, nodeTrajectory] = dijkstra(numberOfNodes,visibilityGraph); %and finally use the dijkstra algorithm for path planning

  figure 3; hold on;
  b=drawCircle(0, 0, L1+L2); %teacher's functions for drawing circles
  hold on;
  c=drawCircle(0, 0, L2-L1);
  hold on;
  %creates the lines defining the prohibited areas
  top_line = createLine([0,L1,1,0]);
  bottom_line = createLine([0,-L1,1,0]);
  drawLine(top_line);
  hold on;
  drawLine(bottom_line);
  center_box=[L2 L2; -L2 L2; -L2 -L2; L2 -L2];
  drawPolygon(center_box);
  hold on;


  Points_path=zeros(size(nodeTrajectory(1,2))+1,2);
  drawPoint(S, 'ro');

  for i=1:size(nodeTrajectory, 2)
    drawPoint(points2D(nodeTrajectory(1, i), 1:2), 'r');
    Points_path(i, 1:2)=points2D(nodeTrajectory(1, i), 1:2);
  endfor

  drawPoint(G, 'ro');

  for j=2:size(Points_path, 1)
    L=createEdge(Points_path(j-1, 1:2), Points_path(j, 1:2));
    drawEdge(L, 'r');
  endfor
  indexClosestPointFinal = findClosestPoint(G,Points_path);
  L=createEdge(Points_path(indexClosestPointFinal, 1:2), G);
  drawEdge(L, 'r');

endfunction

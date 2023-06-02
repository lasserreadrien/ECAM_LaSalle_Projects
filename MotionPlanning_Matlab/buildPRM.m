## Author: adril <adril@LAPTOP-EJ1AIJHT>
## Created: 2022-12-06
## For more info:
## Check library matgeom: https://octave.sourceforge.io/matgeom/overview.html

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%function buildPRM (rangeQ1Q2, nbPoints, L1, L2, MapFilename)
%
% Task:
%The goal of this function is to use the Probabilistic RoadMaps procedure,
%to build a map of both the Cartesian space and the C-space representations,
%with the number of points chosen by the user to establish the different
%options for the path to follow, to get to a specific goal (cartesian point)
%
% Inputs:
%	- rangeQ1Q2 : range of values (in degrees) acceptable for joints Q1 and Q2
% - nbPoints : number of points required
% - L1, L2 : lengths of the links (in m)
% - MapFilename : the name of the file to be saved for the map
%
% Outputs:
%	- .mat file in the workspace
%
% Adrien Lasserre (adrien.lasserre@ecam.fr) & Gwenn Durpoix-Espinasson (g.durpoix-espinasson@ecam.fr)
% 06/12/2022
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function buildPRM (rangeQ1Q2, nbPoints, L1, L2, MapFilename)

  hold off;
  i = 1; %initialize
  Points=zeros(2, nbPoints); %matrix of 2xnbPoints
  MatrixOfLinks=zeros(nbPoints, nbPoints);
  alpha=[0;0];
  d=[0;0];
  a=[L1;L2];
  jointNumber=[1;2];

  figure 1; hold on;
  b=drawCircle(0, 0, L1+L2); %teacher's functions for drawing circles
  ## used for drawing the robot definition
  hold on;
  c=drawCircle(0, 0, L2-L1);
  hold on;
  %creates the lines defining the prohibited areas
  top_line = createLine([0,L1,1,0]);
  bottom_line = createLine([0,-L1,1,0]);
  drawLine(top_line);
  hold on;
  drawLine(bottom_line);
  %creates the central box that's prohibited
  center_box=[L2 L2; -L2 L2; -L2 -L2; L2 -L2];
  drawPolygon(center_box);
  hold on;

  poly_a=circleToPolygon([0 0 L2-L1], 32);%create a polygon for matgeom with the circle info (smaller one)
  poly_b=circleToPolygon([0 0 L1+L2], 32);%bigger one radius=3

  while i <= nbPoints % will work until 10 valid points are found (no intersection)

    %creates random angle values for Q1 and Q2
    Q=[rand()*(rangeQ1Q2(1,2)-rangeQ1Q2(1,1))+rangeQ1Q2(1,1);rand()*(rangeQ1Q2(2,2)-rangeQ1Q2(2,1))+rangeQ1Q2(2,1)];
    theta=[Q(1,1);Q(2,1)];

    OutOfRange=0; %set the boolean

    bTee=dh2ForwardKinematics(theta, d, a, alpha, jointNumber); %FW kinematics
    jTee=bTee(1:2, 4); %only retrieve the x and y (2D) values

    %checks if the end effector is in a prohibited zone
    if (jTee(2,1)>=L1)
      OutOfRange=1; %is not valid if in that area
    elseif (jTee(2,1)<=-L1)
      OutOfRange=1;
    elseif (abs(jTee(1,1)) <= L2 && abs(jTee(2,1)) <=L2)
      OutOfRange=1;
    endif

    if (OutOfRange==0)
      Q_storage(1:2, i)=Q;
      Points(1:2, i)=jTee; %assign the current random point to the Points matrix
      MatrixOfLinks(i, i)=1; %it doesnt intersect with the obstacles when compared to itself

      if i == 1
        drawPoint(jTee(1,1), jTee(2,1)); %draw the point
      endif

      if i>=2 %i=1 useless

        for j=1:i %compare with the other points to check if it can be connected to them

          intersect=0; %set no intersection to start with
          %takes the coordinates of the current and past points
          current_point=[Points(1, i), Points(2, i)];
          previous_point=[Points(1, j), Points(2, j)];

          %creates a line-segment between the points before checking if that line-segment instersects an obstacle
          L = createEdge(current_point, previous_point);
          %intersect returns a vector of points of intersection meaning that if it is empty, there is no intersection
          if (isempty(intersectEdgePolygon(L, poly_a))!=1 | isempty(intersectEdgePolygon(L, poly_b))!=1 | isempty(intersectEdgePolygon(L, center_box))!=1)
            intersect=1; % intersection happenned

          else %if there is no intersection, plot the line-segment and the point and adds it to the list of valid points
            MatrixOfLinks(i, j)=1;
            MatrixOfLinks(j,i)=1;
            hold on;%plotting the line
            drawEdge(L);
            hold on;
            drawPoint(jTee(1,1), jTee(2,1)); %draw the point
          endif
        endfor
      endif
      i= i + 1;
    endif
  end

  MatrixOfLinks;
  Points %display the points vector to check on the graph
  qGraph (Q_storage, nbPoints, MatrixOfLinks)

  %saves the results into a .mat file
  save(strcat(MapFilename,'.mat'), 'Points', 'MatrixOfLinks')

end

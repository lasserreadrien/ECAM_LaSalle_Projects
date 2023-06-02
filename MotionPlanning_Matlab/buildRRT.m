## Author: adril <adril@LAPTOP-EJ1AIJHT>
## Created: 2023-01-08

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%function buildRRT(rangeQ1Q2, nbPoints, L1, L2, fixedLength, start, goal)
%
% Task: Use the RRT method to reach a desired goal on the cartesian space
%
% Inputs:
%	- rangeQ1Q2 : range of values (in degrees) acceptable for joints Q1 and Q2
% - nbPoints : number of points required
% - L1, L2 : lengths of the links (in m)
% - fixedLength : length of the short link
% - start : starting point
% - goal : ending point
%
% Outputs:
%	- None
%
% Adrien Lasserre (adrien.lasserre@ecam.fr) &
% Gwenn Durpoix-Espinasson (g.durpoix-espinasson@ecam.fr)
% 08/01/2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function buildRRT(rangeQ1Q2, nbPoints, L1, L2, fixedLength, start, goal)

  hold off;
  i = 1; %while
  nbPoints=nbPoints+2;%adding the starting point
  %Points=zeros(nbPoints, 2);
  %MatrixOfLinks=zeros(nbPoints, nbPoints);
  alpha=[0;0];
  d=[0;0];
  a=[L1;L2];
  jointNumber=1;

  figure 1; hold on;
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

  poly_a=circleToPolygon([0 0 L2-L1], 32);%create a polygon for matgeom with the circle info (smaller one)
  poly_b=circleToPolygon([0 0 L1+L2], 32);%bigger one radius=3

  %set the starting point
  Points(1,1:2) = start;

  %draw the start and goal points
  drawPoint(start(1, 1), start(1,2));
  drawPoint(goal(1, 1), goal(1,2));

  while i <= nbPoints

    Q=[rand()*(rangeQ1Q2(1,2)-rangeQ1Q2(1,1))+rangeQ1Q2(1,1);rand()*(rangeQ1Q2(2,2)-rangeQ1Q2(2,1))+rangeQ1Q2(2,1)];
    theta=[Q(1,1);Q(2,1)];
    Q_storage(1:2, i)=Q;
    OutOfRange=0; %set the boolean
    intersect=0;

    bTee=dh2ForwardKinematics(theta, d, a, alpha, jointNumber); %FW kinematics
    jTee=bTee(1:2, 4); %only retrieve the x and y (2D) values
    jTee=jTee';

    index=findClosestPoint(jTee, Points);
    dx=jTee(1,1)-Points(index, 1);
    dy=jTee(1,2)-Points(index, 2);
    E=sqrt(dx^2+dy^2);
    dx=(dx)*fixedLength/E;
    dy=(dy)*fixedLength/E;
    L = createEdge(Points(index, :), [Points(index, 1)+dx, Points(index,2)+dy]);

    if ((Points(index,2)+dy)>=L1)
      OutOfRange=1; %is not valid if in that area
    elseif ((Points(index,2)+dy)<=-L1)
      OutOfRange=1;
    elseif (abs((Points(index,1)+dx)) <= L2 && abs((Points(index,2)+dy)) <=L2)
      OutOfRange=1;
    endif

    if (OutOfRange==0)

      if (isempty(intersectEdgePolygon(L, poly_a))!=1 | isempty(intersectEdgePolygon(L, poly_b))!=1 | isempty(intersectEdgePolygon(L, center_box))!=1)
        intersect=1; % intersection happenned
        disp('Intersect')
      else %if there is no intersection, plot the line-segment and the point and adds it to the list of valid points
        hold on;%plotting the line
        drawEdge(L);
        MatrixOfLinks(i, i)=1;
        MatrixOfLinks(i, index)=1;
        MatrixOfLinks(index, i)=1;
        Points(i, 1:2)=[Points(index, 1)+dx, Points(index,2)+dy];
        hold on;
        drawPoint(Points(i, 1), Points(i,2)); %draw the point
        intersect=0;
        L = createEdge(goal, Points(i, 1:2));

        if (isempty(intersectEdgePolygon(L, poly_a))!=1 | isempty(intersectEdgePolygon(L, poly_b))!=1 | isempty(intersectEdgePolygon(L, center_box))!=1)
          intersect=1;
        else
          drawEdge(L);
          MatrixOfLinks(i+1, i+1)=1;
          MatrixOfLinks(i+1,i)=1;
          MatrixOfLinks(i,i+1)=1;
          i=i+1;
          break;
        endif

        i=i+1;
      endif
    endif
  endwhile

##  Write the path planning here

##  Start from goal, towards start, going to the previous point each time
##  check everytime if the start is not already accessible, and if so draw a line
##  prendre le goal, trouver le point auquel il est attach, y aller

  index_goal = columns(MatrixOfLinks);
  index = index_goal;

##Draw the clean figure
  figure 2; hold on;
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

  poly_a=circleToPolygon([0 0 L2-L1], 32);%create a polygon for matgeom with the circle info (smaller one)
  poly_b=circleToPolygon([0 0 L1+L2], 32);%bigger one radius=3

  %set the starting point
  %Points(1,1:2) = start;

  %draw the start and goal points
  %drawPoint(start(1, 1), start(1,2));
  Points_2(1, 1:2)=goal;
  drawPoint(Points_2(1, 1:2), 'ro');
  z=1;
  MatrixOfLinks
  while Points_2(z, 1:2)!=Points(1, 1:2)
##  Start the path planning
##Check if we can reach the start
    L = createEdge(start, Points_2(z, 1:2));
    if (isempty(intersectEdgePolygon(L, poly_a))!=1 | isempty(intersectEdgePolygon(L, poly_b))!=1 | isempty(intersectEdgePolygon(L, center_box))!=1)
      intersect=1;
    else
      drawEdge(L, 'r');
      break;
    endif
  ##If not, then check the next point
    for i=1:index-1
      if MatrixOfLinks(index, i)==1
        disp('found a point connected');
        Points_2(z+1, 1:2)=Points(i, 1:2);
        disp('new point added, and drawn');
        drawPoint(Points_2(z+1, 1:2), 'r');
        L=createEdge(Points_2(z+1, 1:2), Points_2(z, 1:2));
        drawEdge(L, 'r');
        index=i;
        break;
      endif
    endfor
    z=z+1;
    disp('next step')
  endwhile

  drawPoint(start, 'ro');

endfunction

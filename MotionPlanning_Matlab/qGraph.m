## Author: adril <adril@LAPTOP-EJ1AIJHT>
## Created: 2023-01-10

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%function qGraph (Q, nbPoints, mat)
%
% Task: Create the C-Space graph for the PRM method, with the selected joint
%       values, that weren't outside of bounds.
%
% Inputs:
%	- Q : matrix that stores the different joint values for each point.
% - nbPoints : number of points required
% - mat : the Matrix of links, that stores the information on which point is
%         linked to which other one.
%
% Outputs:
%	- None
%
% Adrien Lasserre (adrien.lasserre@ecam.fr) & Gwenn Durpoix-Espinasson (g.durpoix-espinasson@ecam.fr)
% 10/01/2023
%
% Check library matgeom: https://octave.sourceforge.io/matgeom/overview.html
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function qGraph (Q, nbPoints, mat)
  hold off;
  figure 2 %launch a new figure
  for i=1:nbPoints %double for loop to check every point for every link
    for j=1:nbPoints
      drawPoint(Q(1,i), Q(2,i)); %draw the point in C-space
      current_point=[Q(1, i), Q(2, i)]; %change the current point
      previous_point=[Q(1, j), Q(2, j)]; %change the previous point
      if mat(i, j)==1 & mat(j, i)==1 %if there is a link between the two
        L = createEdge(current_point, previous_point); %then create a line between both
        hold on;
        drawEdge(L); %and draw it on the figure.
      endif
    endfor
  endfor
endfunction

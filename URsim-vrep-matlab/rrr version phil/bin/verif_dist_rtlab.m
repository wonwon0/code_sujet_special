function [d_min, pose_prox_out]=verif_dist_rtlab(surface1,surface2,centroide,rayonProxy,pose_ac)
%--------------------------------------------ADAPTEUR À ENBLEVER POUR
%METTRE DANS RTLAB---------------------------------------------------
pose_ac=pose_ac';

%#codegen
pose=pose_ac';
d_min=0;
pose_prox=[10 10 10];
%verification proximité:
    base=surface1;
    top=surface2;
    ratio_hauteure=(pose(3)-base(1,3))/(top(1,3)-base(1,3));
    section_inter=(1-ratio_hauteure)*base+(ratio_hauteure)*top;
    if rayonProxy>norm((pose-centroide))
        %si on est en haut ou en bas de l'objet:
        if (pose(3)<base(1,3));
            [d_min, x_d_min, y_d_min, ~, ~, ~, ~, ~, ~, ~]=p_poly_dist(pose(1), pose(2), base(:,1) , base(:,2),true);
            if (dot([x_d_min(1) y_d_min(1)]-[centroide(1),centroide(2)],[pose(1),pose(2)]-[x_d_min(1) y_d_min(1)])<=0)
                %On est directement en dessous de l'obstacle
                pose_prox=[pose(1) pose(2) base(1,3)];
                d_min(1)=sqrt((pose(3)-base(1,3))^2);
            else
                %On est en dessous et en biais de l'objet
                pose_prox=[x_d_min(1) y_d_min(1) base(1,3)];
                d_min(1)=sqrt(d_min(1)^2+(pose(3)-base(1,3))^2);
            end
        elseif (pose(3)>top(1,3));
            [d_min, x_d_min, y_d_min, ~, ~, ~, ~, ~, ~, ~]=p_poly_dist(pose(1), pose(2), top(:,1) , top(:,2),true);
            if (dot([x_d_min(1) y_d_min(1)]-[centroide(1),centroide(2)],[pose(1),pose(2)]-[x_d_min(1) y_d_min(1)])<=0)
                %On est directement au dessus de l'obstacle
                pose_prox=[pose(1) pose(2) top(1,3)];
                d_min(1)=sqrt((pose(3)-top(1,3))^2);
            else
                %On est directement au dessus et en biais de l'obstacle
                pose_prox=[x_d_min(1) y_d_min(1) top(1,3)];
                d_min(1)=sqrt(d_min(1)^2+(pose(3)-top(1,3))^2);
            end
        end
        if pose(3)>base(1,3) && pose(3)<top(1,3)
            %si on est entre le plan de dessus de l'objet et le plan de dessous
            [d_min, x_d_min, y_d_min, ~, ~, ~, ~, ~, ~, ~]=p_poly_dist(pose(1), pose(2), section_inter(:,1) , section_inter(:,2),true);
            if (dot([x_d_min(1) y_d_min(1)]-[centroide(1),centroide(2)],[pose(1),pose(2)]-[x_d_min(1) y_d_min(1)])<=0)
                %On est dans l'objet
                pose_prox=pose;
            else
                pose_prox=[x_d_min(1) y_d_min(1) pose(3)];
            end
        end
    else
        d_min=10;
        pose_prox=pose;
    end
    pose_prox_out=pose_prox';
    %--------------------------------------------ADAPTEUR À ENBLEVER POUR
%METTRE DANS RTLAB---------------------------------------------------
    pose_prox_out=pose_prox_out';
end
    
    
function [d_min, varargout] = p_poly_dist(xp, yp, xv, yv, varargin)
%#codegen
%p_poly_dist Find minimum distances from points to a polyline or to a 
% closed polygon.
%
% Description:
% Compute the distances from each one of a set of np points p(1), p(2),...
% p(np) on a 2D plane to a polyline or a closed polygon. Polyline is 
% defined as a set of nv-1 segments connecting nv ordered vertices v(1), 
% v(2), ..., v(nv). The polyline can optionally be treated as a closed
% polygon.
% Distance from point j to a segment k is defined as a distance from this
% point to a straight line passing through vertices v(k) and v(k+1), when
% the projection of point j on this line falls INSIDE segment k; and to
% the closest of v(k) or v(k+1) vertices, when the projection falls OUTSIDE
% segment k. Distance from point j to a polyline is defined as a minimum of
% this point's distances to all segments.
% In a case when the projected point fall OUTSIDE of all polyline segments,
% the distance to a closest vertex of the polyline is returned
%
% Input arguments:
% Required:
% [d_min, varargout] = p_poly_dist(xp, yp, xv, yv)
% xp - vector of points X coordinates (1 X np)
% yp - vector of points Y coordinates (1 X np)
% xv - vector of polygon vertices' X coordinates (1 X nv)
% yv - vector of polygon vertices' Y coordinates (1 X nv)
%
% Optional:
% [d_min, varargout] = p_poly_dist(xp, yp, xv, yv, find_in_out)
%
% find_in_out - logical flag. When true, the polyline is treated as a
% closed polygon, and each input point is checked wether it is inside or 
% outside of this polygon. In such case, an open polyline is automatically 
% closed by adding a last point identical to a first one. 
% Note: when this function is called with ver. 1.0 signature, i.e:
% d = p_poly_dist(xp, yp, xv, yv)
% the flag find_in_out is assumed to be true, to keep the functionality
% compatible with a previous version.
%
% Output arguments:
% Required:
% d_min = p_poly_dist(xp, yp, xv, yv, varargin)
%
% d_min - vector of distances (1 X np) from points to polyline. This is 
% defined as either a distance from a point to it's projection on a line 
% that passes through a pair of consecutive polyline vertices, when this 
% projection falls inside a segment; or as a distance from a point to a 
% closest vertex, when the projection falls outside of a segment. When 
% find_in_out input flag is true, the polyline is assumed to be a closed 
% polygon, and distances of points INSIDE this polygon are defined as 
% negative.
% 
% Optional:
% [d_min, x_d_min, y_d_min, is_vertex, xc, yc, idx_c, Cer, Ppr] = 
%       p_poly_dist(xp, yp, xv, yv, varargin);
%
% x_d_min - vector (1 X np) of X coordinates of the closest points of
% polyline. 
%
% y_d_min - vector (1 X np) of Y coordinates of the closest points of
% polyline. 
%
% is_vertex - logical vector (1 X np). If is_vertex(j)==true, the closest
% polyline point to a point (xp(j), yp(j)) is a vertex. Otherwise, this
% point is a projection on polyline's segment.
%
% idx_c - vector (1 X np) of indices of segments that contain the closest
% point. For instance,  idx_c(2) == 4 means that the polyline point closest
% to point 2 belongs to segment 4
%
% xc - an array (np X nv-1) containing X coordinates of all projected
% points. xc(j,k) is an X coordinate of a projection of point j on a
% segment k
%
% yc - an array (np X nv-1) containing Y coordinates of all projected
% points. yc(j,k) is Y coordinate of a projection of point j on a
% segment k
%
% is_in_seg - logical array (np X nv-1). If is_in_seg(j,k) == true, the
% projected point j with coordinates (xc(j,k), yc(j,k)) lies INSIDE the
% segment k
%
% Cer - a 3D array (2 X 2 X nv-1). Each 2 X 2 slice represents a rotation
% matrix from an input Cartesian coordinate system to a system defined
% by polyline segments.
%
% Ppr - 3D array of size 2 X np X (nv-1). Ppr(1,j,k) is an X coordinate
% of point j in coordinate systems defined by segment k. Ppr(2,j,k) is its
% Y coordinate.
%
% Routines: p_poly_dist.m
% Revision history:
% Oct 2, 2015 - version 2.0 (Michael Yoshpe). The original ver. 1.0 
% function was completely redesigned. The main changes introduced in 
% ver. 2.0:
% 1. Distances to polyline (instead of a closed polygon in ver. 1.0) are 
% returned. The polyline can optionally be treated as a closed polygon.
% 2. Distances from multiple points to the same polyline can be found
% 3. The algorithm for finding the closest points is now based on 
% coordinate system transformation. The new algorithm avoids numerical 
% problems that ver. 1.0 algorithm could experience in "ill-conditioned" 
% cases.
% 4. Many optional output variables were added. In particular, the closest
% points on polyline can be returned.
% 5. Added input validity checks
% 7/9/2006  - case when all projections are outside of polygon ribs
% 23/5/2004 - created by Michael Yoshpe 
% Remarks:
%**************************************************************************

find_in_out = false;

if(nargin >= 5)
   find_in_out = varargin{1};  
elseif((nargin==4) && (nargout==1)) % mimic ver. 1.0 functionality
   find_in_out = true;
end

% number of points and number of vertices in polyline
nv = length(xv);
np = length(xp);

if(nv < 2)
   error('Polyline must have at least 2 vertices');
end

if((find_in_out == true) && (nv < 3))
   error('Polygon must have at least 3 vertices');
end

% if finding wether the points are inside or outsite the polygon is
% required, make sure the verices represent a closed polygon
% if(find_in_out)
%    % If (xv,yv) is not closed, close it.
%    nv = length(xv);
%    if ((xv(1) ~= xv(nv)) || (yv(1) ~= yv(nv)))
%       xv = [xv(:)' xv(1)];
%       yv = [yv(:)' yv(1)];
%       nv = nv + 1;
%    end
% end

% Cartesian coordinates of the polyline vertices
Pv = [xv(:) yv(:)];

% Cartesian coordinates of the given points
Pp = [xp(:) yp(:)];

% matrix of distances between all points and all vertices
% dpv(j,k) - distance from point j to vertex k
dpv = hypot((repmat(Pv(:,1)', [np 1])-repmat(Pp(:,1), [1 nv])),...
                 (repmat(Pv(:,2)', [np 1])-repmat(Pp(:,2), [1 nv])));

% Find the vector of minimum distances to vertices. 
[dpv_min, I_dpv_min] = min(abs(dpv),[],2);

% coordinates of consecutive vertices
P1 = Pv(1:(end-1),:);
P2 = Pv(2:end,:);
dv = P2 - P1;

% vector of distances between each pair of consecutive vertices
vds = hypot(dv(:,1), dv(:,2));

% check for identical points
idx = find(vds < 10*eps);
% if(~isempty(idx))
%    error(['Points ' num2str(idx) ' of the polyline are identical']);
% end

% check for a case when closed polygon's vertices lie on a stright line, 
% i.e. the distance between the last and first vertices is equal to the sum
% of all segments except the last
if(find_in_out)
   s = cumsum(vds);
   if((s(end-1) - vds(end)) < 10*eps)
      error('Polygon vertices should not lie on a straight line');
   end
end

% Each pair of consecutive vertices P1(j), P2(j) defines a rotated 
% coordinate system with origin at P1(j), and x axis along the vector 
% (P2(j)-P1(j)). 
% Build the rotation matrix Cer from original to rotated system
ctheta = dv(:,1)./vds;
stheta = dv(:,2)./vds;
Cer = zeros(2,2,nv-1);
Cer(1,1,:) = ctheta;
Cer(1,2,:) = stheta;
Cer(2,1,:) = -stheta;
Cer(2,2,:) = ctheta;

% Build the origin translation vector P1r in rotated frame by rotating the
% P1 vector
P1r = [(ctheta.*P1(:,1) + stheta.*P1(:,2)),...
       -stheta.*P1(:,1) + ctheta.*P1(:,2)];

Cer21 = zeros(2, nv-1);
Cer22 = zeros(2, nv-1);

Cer21(:,:) = Cer(1,:,:);
Cer22(:,:) = Cer(2,:,:);

% Ppr is a 3D array of size 2 * np * (nv-1). Ppr(1,j,k) is an X coordinate
% of point j in coordinate systems defined by segment k. Ppr(2,j,k) is its
% Y coordinate.

% Rotation. Doing it manually, since Matlab cannot multiply 2D slices of ND
% arrays.
Ppr1 = Pp*Cer21;
Ppr2 = Pp*Cer22;
% Ppr(1,:,:) = Pp*Cer21;
% Ppr(2,:,:) = Pp*Cer22;
a1=permute(repmat(P1r(:,1), [1 1 np]), [2 3 1]);
a2=permute(repmat(P1r(:,2), [1 1 np]), [2 3 1]);
a1=squeeze(a1(1,:,:));
a2=squeeze(a2(1,:,:));
% translation
Ppr1 = Ppr1 - a1';
Ppr2 = Ppr2 - a2';

% Pcr is a 3D array of size 2 * np * (nv-1) that holds the projections of
% points on X axis of rotated coordinate systems. Pcr(1,j,k) is an X
% coordinate of point j in coordinate systems defined by segment k.
% Pcr(2,j,k) is its Y coordinate, which is identically zero for projected
% point.
Pcr = zeros(2,size(Ppr1,1),size(Ppr1,2));
Pcr(1, :, :) = Ppr1;
Pcr(2, :, :) = 0;

% Cre is a rotation matrix from rotated to original system
Cre = permute(Cer, [2 1 3]);

% Pce is a 3D array of size 2 * np * (nv-1) that holds the projections of
% points on a segment in original coordinate systems. Pce(1,j,k) is an X
% coordinate of the projection of point j on segment k.
% Pce(2,j,k) is its Y coordinate
Pce = zeros(2,np,(nv-1));
Pce(1,:,:) = Pcr(1,:,:).*repmat(Cre(1,1,:), [1 np 1]) + ...
             Pcr(2,:,:).*repmat(Cre(1,2,:), [1 np 1]);
Pce(2,:,:) = Pcr(1,:,:).*repmat(Cre(2,1,:), [1 np 1]) + ...
             Pcr(2,:,:).*repmat(Cre(2,2,:), [1 np 1]);
          
% Adding the P1 vector
Pce(1,:,:) = Pce(1,:,:) + permute(repmat(P1(:,1), [1 1 np]), [2 3 1]);
Pce(2,:,:) = Pce(2,:,:) + permute(repmat(P1(:,2), [1 1 np]), [2 3 1]);

% x and y coordinates of the projected (cross-over) points in original
% coordinate frame
xc = zeros(np, (nv-1));
yc = zeros(np, (nv-1));
xc(:,:) = Pce(1,:,:);
yc(:,:) = Pce(2,:,:);

r = zeros(np,(nv-1));
cr = zeros(np,(nv-1));
r(:,:) = Ppr1;
cr(:,:) = Ppr2;

% Find the projections that fall inside the segments
is_in_seg = (r>0) & (r<repmat(vds(:)', [np  1 1]));

% find the minimum distances from points to their projections that fall
% inside the segments (note, that for some points these might not exist,
% which means that closest points are vertices)
B = NaN(np,nv-1);
B(is_in_seg) = cr(is_in_seg);

[cr_min, I_cr_min] = min(abs(B),[],2);

% Build the logical index which is true for members that are vertices,
% false otherwise. Case of NaN in cr_min means that projected point falls
% outside of a segment, so in such case the closest point is vertex.

% point's projection falls outside of ALL segments
cond1 = isnan(cr_min); 

% point's projection falls inside some segments, find segments for which
% the minimum distance to vertex is smaller than to projection
cond2 = ((I_cr_min ~= I_dpv_min) & (cr_min - dpv_min)> 0);

    is_vertex = (cond1 | cond2);


% build the minimum distances vector
d_min = cr_min;
if(is_vertex)
    d_min = dpv_min;
end
% mimic the functionality of ver. 1.0 - make all distances negative for
% points INSIDE the polygon
if(find_in_out)
   in = inpoly([xp, yp], [xv, yv]);
       if(in)
            d_min = -d_min;
       end
end

% initialize the vectors of minimum distances to the closest vertices
nz = max(np, nv);

vtmp = zeros(nz, 1);
vtmp(1:nv) = xv(:);
x_d_min = vtmp(I_dpv_min);

vtmp = zeros(nz, 1);
vtmp(1:nv) = yv(:);
y_d_min = vtmp(I_dpv_min);

% replace the minimum distances with those to projected points that fall
% inside the segments

%idx_pr = sub2ind(size(xc), find(~is_vertex), I_cr_min(~is_vertex));
if(~is_vertex)
    x_d_min = xc(I_cr_min);
    y_d_min = yc(I_cr_min);
end

% find the indices of segments that contain the closest points
% note that I_dpv_min contains indices of closest POINTS. To find the 
% indices of closest SEGMENTS, we have to substract 1
idx_c = I_dpv_min-1; 
[ii,jj] = ind2sub(size(xc), I_cr_min);
idx_c(ii) = jj;

% assign optional outputs
switch nargout
   case 0
   case 1
   case 2
      varargout{1} = x_d_min;
   case 3
      varargout{1} = x_d_min;
      varargout{2} = y_d_min;
   case 4
      varargout{1} = x_d_min;
      varargout{2} = y_d_min;
      varargout{3} = is_vertex;
   case 5
      varargout{1} = x_d_min;
      varargout{2} = y_d_min;
      varargout{3} = is_vertex;
      varargout{4} = idx_c;
   case 6
      varargout{1} = x_d_min;
      varargout{2} = y_d_min;
      varargout{3} = is_vertex;
      varargout{4} = idx_c;
      varargout{5} = xc;
   case 7
      varargout{1} = x_d_min;
      varargout{2} = y_d_min;
      varargout{3} = is_vertex;
      varargout{4} = idx_c;
      varargout{5} = xc;
      varargout{6} = yc;
   case 8
      varargout{1} = x_d_min;
      varargout{2} = y_d_min;
      varargout{3} = is_vertex;
      varargout{4} = idx_c;
      varargout{5} = xc;
      varargout{6} = yc;
      varargout{7} = is_in_seg;
   case 9
      varargout{1} = x_d_min;
      varargout{2} = y_d_min;
      varargout{3} = is_vertex;
      varargout{4} = idx_c;
      varargout{5} = xc;
      varargout{6} = yc;
      varargout{7} = is_in_seg;      
      varargout{8} = Cer;
   case 10
      varargout{1} = x_d_min;
      varargout{2} = y_d_min;
      varargout{3} = is_vertex;
      varargout{4} = idx_c;
      varargout{5} = xc;
      varargout{6} = yc;
      varargout{7} = is_in_seg;            
      varargout{8} = Cer;
      varargout{9} = Ppr1;      
   otherwise
      error('Invalid number of output arguments, must be between 1 and 9');
end

end


function [cn,on] = inpoly(p,node,edge,TOL)
%#codegen
%  INPOLY: Point-in-polygon testing.
%
% Determine whether a series of points lie within the bounds of a polygon
% in the 2D plane. General non-convex, multiply-connected polygonal
% regions can be handled.
%
% SHORT SYNTAX:
%
%   in = inpoly(p,node);
%
%   p   : The points to be tested as an Nx2 array [x1 y1; x2 y2; etc].
%   node: The vertices of the polygon as an Mx2 array [X1 Y1; X2 Y2; etc].
%         The standard syntax assumes that the vertices are specified in
%         consecutive order.
%
%   in  : An Nx1 logical array with IN(i) = TRUE if P(i,:) lies within the
%         region.
%
% LONG SYNTAX:
%
%  [in,on] = inpoly(p,node,edge);
%
%  edge: An Mx2 array of polygon edges, specified as connections between
%        the vertices in NODE: [n1 n2; n3 n4; etc]. The vertices in NODE
%        do not need to be specified in connsecutive order when using the
%        extended syntax.
%
%  on  : An Nx1 logical array with ON(i) = TRUE if P(i,:) lies on a
%        polygon edge. (A tolerance is used to deal with numerical
%        precision, so that points within a distance of
%        eps^0.8*norm(node(:),inf) from a polygon edge are considered "on"
%        the edge.
%
% EXAMPLE:
%
%   polydemo;       % Will run a few examples
%
% See also INPOLYGON

% The algorithm is based on the crossing number test, which counts the
% number of times a line that extends from each point past the right-most
% region of the polygon intersects with a polygon edge. Points with odd
% counts are inside. A simple implementation of this method requires each
% wall intersection be checked for each point, resulting in an O(N*M)
% operation count.
%
% This implementation does better in 2 ways:
%
%   1. The test points are sorted by y-value and a binary search is used to
%      find the first point in the list that has a chance of intersecting
%      with a given wall. The sorted list is also used to determine when we
%      have reached the last point in the list that has a chance of
%      intersection. This means that in general only a small portion of
%      points are checked for each wall, rather than the whole set.
%
%   2. The intersection test is simplified by first checking against the
%      bounding box for a given wall segment. Checking against the bbox is
%      an inexpensive alternative to the full intersection test and allows
%      us to take a number of shortcuts, minimising the number of times the
%      full test needs to be done.
%
%   Darren Engwirda: 2005-2007
%   Email          : d_engwirda@hotmail.com
%   Last updated   : 23/11/2007 with MATLAB 7.0
%
% Problems or suggestions? Email me.

%% ERROR CHECKING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if nargin<4
   TOL = 1.0e-12;
   if nargin<3
      nnode = size(node,1);
      edge = [(1:nnode-1)' (2:nnode)'; nnode 1];
      if nargin<2
         error('Insufficient inputs');
      end
   end
end

if size(p,2)~=2
   error('P must be an Nx2 array.');
end
if size(node,2)~=2
   error('NODE must be an Mx2 array.');
end
if size(edge,2)~=2
   error('EDGE must be an Mx2 array.');
end
if max(edge(:))>nnode || any(edge(:)<1)
   error('Invalid EDGE.');
end

%% PRE-PROCESSING
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n  = size(p,1);
nc = size(edge,1);

% Choose the direction with the biggest range as the "y-coordinate" for the
% test. This should ensure that the sorting is done along the best
% direction for long and skinny problems wrt either the x or y axes.
dxy = max(p,[],1)-min(p,[],1);
if dxy(1)>dxy(2)
   % Flip co-ords if x range is bigger
   p = p(:,[2,1]);
   node = node(:,[2,1]);
end
tol = TOL*min(dxy);

% Sort test points by y-value
[y,i] = sort(p(:,2));
x = p(i,1);

%% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
cn = false(n,1);     % Because we're dealing with mod(cn,2) we don't have
                     % to actually increment the crossing number, we can
                     % just flip a logical at each intersection (faster!)
on = cn;
for k = 1:nc         % Loop through edges

   % Nodes in current edge
   n1 = edge(k,1);
   n2 = edge(k,2);

   % Endpoints - sorted so that [x1,y1] & [x2,y2] has y1<=y2
   %           - also get xmin = min(x1,x2), xmax = max(x1,x2)
   y1 = node(n1,2);
   y2 = node(n2,2);
   if y1<y2
      x1 = node(n1,1);
      x2 = node(n2,1);
   else
      yt = y1;
      y1 = y2;
      y2 = yt;
      x1 = node(n2,1);
      x2 = node(n1,1);
   end
   if x1>x2
      xmin = x2;
      xmax = x1;
   else
      xmin = x1;
      xmax = x2;
   end

   % Binary search to find first point with y<=y1 for current edge
   if y(1)>=y1
      start = 1;
   elseif y(n)<y1
      start = n+1;       
   else
      lower = 1;
      upper = n;
      for j = 1:n
         start = round(0.5*(lower+upper));
         if y(start)<y1
            lower = start;
         elseif y(start-1)<y1
            break;
         else
            upper = start;
         end
      end
   end

   % Loop through points
   for j = start:n
      % Check the bounding-box for the edge before doing the intersection
      % test. Take shortcuts wherever possible!

      Y = y(j);   % Do the array look-up once & make a temp scalar
      if Y<=y2
         X = x(j);   % Do the array look-up once & make a temp scalar
         if X>=xmin
            if X<=xmax

               % Check if we're "on" the edge
               on(j) = on(j) || (abs((y2-Y)*(x1-X)-(y1-Y)*(x2-X))<tol);

               % Do the actual intersection test
               if (Y<y2) && ((y2-y1)*(X-x1)<(Y-y1)*(x2-x1))
                  cn(j) = ~cn(j);
               end

            end
         elseif Y<y2   % Deal with points exactly at vertices
            % Has to cross edge
            cn(j) = ~cn(j);
         end
      else
         % Due to the sorting, no points with >y
         % value need to be checked
         break
      end
   end

end

% Re-index to undo the sorting
cn(i) = cn|on;
on(i) = on;

end      % inpoly()



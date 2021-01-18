module prism(l, w, h){
       polyhedron(
               points=[[0,0,0], [l,0,0], [l,w,0], [0,w,0], [0,w,h], [l,w,h]],
               faces=[[0,1,2,3],[5,4,3,2],[0,4,5,1],[0,3,4],[5,2,1]]
               );
       
       // preview unfolded (do not include in your function
       z = 0.08;
       separation = 2;
       border = .2;
       //translate([0,w+separation,0])
       //    cube([l,w,z]);
       //translate([0,w+separation+w+border,0])
       //    cube([l,h,z]);
       //translate([0,w+separation+w+border+h+border,0])
       //    cube([l,sqrt(w*w+h*h),z]);
       //translate([l+border,w+separation+w+border+h+border,0])
       //    polyhedron(
       //            points=[[0,0,0],[h,0,0],[0,sqrt(w*w+h*h),0], [0,0,z],[h,0,z],[0,sqrt(w*w+h*h),z]],
       //            faces=[[0,1,2], [3,5,4], [0,3,4,1], [1,4,5,2], [2,5,3,0]]
       //            );
       //translate([0-border,w+separation+w+border+h+border,0])
       //    polyhedron(
       //            points=[[0,0,0],[0-h,0,0],[0,sqrt(w*w+h*h),0], [0,0,z],[0-h,0,z],[0,sqrt(w*w+h*h),z]],
       //            faces=[[1,0,2],[5,3,4],[0,1,4,3],[1,2,5,4],[2,0,3,5]]
       //            );
       }
   
   prism(12, 12, 8);

$fn = 40;

*knob();
*rotate([0, 90, 0]) motor_mount();
*dish();
end_stop();

eps = 1e-3;

module motor_mount() {
	motor_screw_dist = 35;
	motor_screw_offset = 8;
	motor_axle_hole = 9.5;
	motor_screw_r = 2;

	plate_d = 1.5;
	plate_h = motor_screw_dist+10;
	plate_w = 20;

	frame_t = 2;
	frame_h = 25;
	frame_w = 14.4;

	microrax_hole_d = 10;
	microrax_hole_r = 1.7;

	translate([0, 0, plate_d/2])
		plate();

	translate([0, 0, -(frame_h+frame_t)/2]) difference() {
		cube([frame_w, plate_h, frame_h+frame_t], center=true);
		translate([0, 0, frame_t/2])
			union() {
				cube([frame_w+eps, plate_h-2*frame_t, frame_h+eps], center=true);
				translate([0, -microrax_hole_d/2, -frame_h/2-frame_t/2])
					cylinder(r=microrax_hole_r, h=frame_t+eps, center=true);
				translate([0, microrax_hole_d/2, -frame_h/2-frame_t/2])
					cylinder(r=microrax_hole_r, h=frame_t+eps, center=true);
			}
	
	}

	for (i=[-1,1]) translate([i*(frame_t/2+10.4/2), 0, -10/2-frame_h-frame_t])
		difference() {
			cube([frame_t, plate_h, 10.4], center=true);
			for (j=[-1,1])
				translate([0, j*microrax_hole_d/2, 0]) 
					rotate([0, 90, 0]) 
						cylinder(r=microrax_hole_r, h=frame_t+eps,center=true);
		}

	module plate () {
		difference() { 
			translate([-(plate_w-frame_w)/2, 0, 0])
				cube([plate_w, plate_h, plate_d], center=true);
			union() {
				cylinder(r=motor_axle_hole/2, h=plate_d+eps, center=true);
				translate([-motor_screw_offset, motor_screw_dist/2, 0])
					cylinder(r=motor_screw_r, h=plate_d+eps, center=true);
				translate([-motor_screw_offset, -motor_screw_dist/2, 0])
					cylinder(r=motor_screw_r, h=plate_d+eps, center=true);
			}
		}
	}
}	

module dish () {
	dish_r = 50;
	dish_r1 = 8;
	dish_t = 2;
	dish_t1 = 3;

	difference() {
		union() {
			cylinder(r=dish_r, h=dish_t);
			translate([0, 0, dish_t]) cylinder(r=dish_r1, h=dish_t1);
		}
		axle();
	}
}

module end_stop () {
	knob_r = 10.5+2;
	th = 2.5;
	h = 2;
	len = 12;
	w = 3;
	
	translate([0, 0, h/2])
		difference() {
			cylinder(r=knob_r+th, h=h, center=true);
			cylinder(r=knob_r, h=h+eps*10, center=true);
		}
	translate([len/2+knob_r+th, 0, h/2]) {
		cube([len+1, w, h], center=true);
		translate([(len+1)/2-2.5/2, 0, 5/2+h/2])
			cube([2.5, w, 5], center=true);
	}
}

module knob () {
	knob_r1 = 10.45;
	knob_r2 = 10.6;
	knob_t = 2;
	knob_h1 = 7;
	knob_h2 = 5;

	laser_width = 10;
	laser_depth = 5;

	difference() { 
		cylinder(r=knob_r1+knob_t, h=knob_h1+knob_h2);
		union() { 
			axle();
//			translate([0, 0, knob_h2]) cylinder(r1=knob_r1, r2=knob_r2, h=knob_h1);
//			translate([0, 0, knob_h1+knob_h2-laser_depth/2])
//				cube([(2*knob_r1+knob_t)*1.2, laser_width, laser_depth], center=true);
		}
	}
}

module axle () {
	stepper_axle_r = 2.5;
	stepper_axle_d = 3;
	stepper_axle_l = 10;
	eps = 0.2;

	intersection() { 
		cylinder(r=stepper_axle_r+eps, h=stepper_axle_l);
		translate([0, 0, stepper_axle_l/2])
			cube([stepper_axle_d+2*eps, 2*(stepper_axle_r+eps), stepper_axle_l], center=true);
	}
}

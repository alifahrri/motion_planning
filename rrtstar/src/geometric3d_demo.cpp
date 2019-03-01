#include "geometric3d.hpp"

int main(int argc, char** argv)
{
	ros::init(argc,argv,"rrtstar_geometric_3d");
	ros::NodeHandle node;

	std::string file;
	if(node.hasParam("octomap_test_file")) {
		node.getParam("octomap_test_file", file);
	} else {
		file = std::string("/home/fahri/data/fr_078_tidyup.bt");
	}
	octomap::OcTree octree(file);
	// gridmap::Publisher pub_ref(node, octree, "/octomap_full");
	// pub_ref.setFrameID("/map");

	// create rrt instance
	auto geom3d_rrt = Geometric::GeomRRTStar3D(node, octree);
	auto &gridmap = geom3d_rrt.octowrapper;
	auto &publisher = geom3d_rrt.pub;
	publisher.setFrameID("/map");
	// create visualization instance
	auto vis = RRTVisual(node);

	// set sampling volume based on the loaded map
	auto& sampler = geom3d_rrt.sampler;
	auto min = std::array<double,3>();
	auto max = std::array<double,3>();
	octree.getMetricMin(min[0], min[1], min[2]);
	octree.getMetricMax(max[0], max[1], max[2]);
	sampler->set_bound(min, max);

	// placeholder for path color with default value
	std::vector<double> point_colors{1.0,0.0,0.0,1.0};
	std::vector<double> lines_colors{0.0,1.0,0.0,0.5};
	// read rosparam for path color (if any)
	if(node.hasParam("path_red"))
		node.getParam("path_red",lines_colors.at(0));
	if(node.hasParam("path_green"))
		node.getParam("path_green",lines_colors.at(1));
	if(node.hasParam("path_blue"))
		node.getParam("path_blue",lines_colors.at(2));
	if(node.hasParam("path_alpha"))
		node.getParam("path_alpha",lines_colors.at(3));
	if(node.hasParam("min_collision")) {
		double prob;
		node.getParam("min_collision",prob);
		gridmap.set_min_collision_prob(prob);
	}
	// placeholder for starting point
	auto p0 = Geometric::Point3D();
	auto t = std::make_tuple(0.0, 0.0, 0.0);
	std::tie(p0.p[0], p0.p[1], p0.p[2]) = t;
	if(node.hasParam("x"))
		node.getParam("x",p0.p[0]);
	if(node.hasParam("y"))
		node.getParam("y",p0.p[1]);
	if(node.hasParam("z"))
		node.getParam("z",p0.p[2]);
	auto c = Geometric::CostType(0.0);
	// set start state
	geom3d_rrt.setStart(p0, c);

	// auto &tree = Geometric::tree3d;
	// auto &geom_rrt = Geometric::rrtstar_3d;
	// auto start = Geometric::State3D(std::make_shared<Geometric::Point3D>(p0),std::make_shared<Geometric::CostType>(c));
	// geom_rrt.setStart(start);

	auto pub = geom3d_rrt.pub;
	// wait 10 seconds befor publish the map
	ROS_INFO("wait for 10 seconds");
	ros::Duration wait(10.0);
	wait.sleep();
	ROS_INFO("publising map");
	pub.publish();

	auto f = 10.0f;
	// set growing frequency (if any)
	if(node.hasParam("rate"))
		node.getParam("rate",f);
	ros::Rate rate(f);

	// main loop
	while(ros::ok()) {
		// geom_rrt.grow();
		// vis.set_nodes(tree.tree.cloud.states,tree.parent,3,tree.tree.size());
		// geom3d_rrt.rrtstar_3d->grow();
		auto rrt = geom3d_rrt.rrtstar_3d;
		auto tree = geom3d_rrt.tree3d;
		ROS_INFO("growing rrt");
		rrt->grow();

		// visualization stuff
		ROS_INFO("setting visual");
		vis.set_nodes<3>(tree->tree.cloud.states,tree->parent,tree->tree.size(),point_colors,lines_colors);
		vis.delete_all();
		vis.publish();
		vis.clear();

		// rate control
		rate.sleep();
	}
	return 0;
}

#include "tools/geometry.h"

namespace geometry {

	std::vector<drake::geometry::GeometryId> getObstacleGeometries(
			drake::multibody::MultibodyPlant<double>* plant
			)
	{
		std::vector<const drake::multibody::Body<double>*> obstacle_bodies =
			plant->GetBodiesWeldedTo(plant->GetBodyByName("ground"));
		// Collect only obstacles
		for (int i = 0; i < obstacle_bodies.size(); ++i)
		{
			if (obstacle_bodies[i]->name().find("obs") == std::string::npos)
			{
				obstacle_bodies.erase(obstacle_bodies.begin() + i);
				--i;
			}
		}

		// Collect all obstacle geometries
		std::vector<drake::geometry::GeometryId> obstacle_geometries;
		for (int i = 0; i < obstacle_bodies.size(); ++i)
		{
			const std::vector<drake::geometry::GeometryId>& obstacle_geometry =
				plant->GetCollisionGeometriesForBody(*obstacle_bodies[i]);
			for (int j = 0; j < obstacle_geometry.size(); ++j)
			{
				obstacle_geometries.push_back(obstacle_geometry[j]);
			}
		}

		return obstacle_geometries;
	}

	std::vector<Eigen::Matrix3Xd> getObstaclesVertices(
				const drake::geometry::QueryObject<double>* query_object,
				const drake::geometry::SceneGraphInspector<double>* inspector,
				std::vector<drake::geometry::GeometryId> geometry_ids
			)
	{
		std::vector<Eigen::Matrix3Xd> obstacles_vertices;
	
		for (const drake::geometry::GeometryId id : geometry_ids)
		{
			std::optional<drake::geometry::Box> box = geometry::BoxExtractor(
					inspector->GetShape(id)).box();
			const drake::math::RigidTransformd& X_WB = query_object->X_WG(id);
			
			const Eigen::Matrix3d rotation = X_WB.rotation().matrix();
			const Eigen::Vector3d& pos_origin = X_WB.translation();

			Eigen::Matrix3Xd obstacle_vertices = geometry::getVertices(
					box->width() * 0.5, box->depth() * 0.5, box->height() * 0.5,
					rotation, pos_origin);

			obstacles_vertices.push_back(obstacle_vertices);
		}

		return obstacles_vertices;
	}

	Eigen::Matrix3Xd getVertices(
			double x_half_width, double y_half_width, double z_half_width,
			Eigen::Matrix3d rotation, Eigen::Vector3d origin
			)
	{
		// Return axis-aligned bounding-box vertices
		// Order:                +y
		//       3------2          |
		//      /|     /|          |
		//     / 4----/-5          ------  +x
		//    0------1 /          /
		//    |/     |/          /
		//    7------6        +z

		const int num_bbox_points = 8;
		Eigen::RowVectorXd cx(num_bbox_points), cy(num_bbox_points), cz(num_bbox_points);
		cx << -1, 1, 1, -1, -1, 1, 1, -1;
		cy << 1, 1, 1, 1, -1, -1, -1, -1;
		cz << 1, 1, -1, -1, -1, -1, 1, 1;
		cx = cx * x_half_width;
		cy = cy * y_half_width;
		cz = cz * z_half_width;

		Eigen::Matrix3Xd points(3,8);
		points << cx, cy, cz;

		auto rotated_points = rotation * points;
		auto translated_and_rotated_points = rotated_points.colwise() + origin;

		return translated_and_rotated_points;
	}
}

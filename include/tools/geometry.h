#pragma once

#include <memory> // unique_ptr

#include "drake/geometry/geometry_visualization.h"
#include <drake/geometry/scene_graph_inspector.h>
#include <drake/geometry/shape_specification.h>
#include "drake/multibody/plant/multibody_plant.h"


namespace geometry {
	using namespace drake::geometry;

	// Needed to get shape properties
	class BoxExtractor : public ShapeReifier {
	 public:
		explicit BoxExtractor(const Shape& shape) { shape.Reify(this); }

		std::optional<Box> box() const { return box_; }

	 private:
		void ImplementGeometry(const Box& box, void*) override { box_ = box; }
		void ImplementGeometry(const Capsule&, void*) override {}
		void ImplementGeometry(const Cylinder&, void*) override {}
		void ImplementGeometry(const Convex&, void*) override {}
		void ImplementGeometry(const Ellipsoid&, void*) override {}
		void ImplementGeometry(const HalfSpace&, void*) override {}
		void ImplementGeometry(const Mesh&, void*) override {}
		void ImplementGeometry(const Sphere&, void*) override {}

		std::optional<Box> box_{};
	};

	Eigen::Matrix3Xd getVertices(
			double x_half_width, double y_half_width, double z_half_width,
			Eigen::Matrix3d rotation, Eigen::Vector3d origin
			);

	std::vector<drake::geometry::GeometryId> getObstacleGeometries(
			drake::multibody::MultibodyPlant<double>* plant
			);

	std::vector<Eigen::Matrix3Xd> getObstaclesVertices(
				const drake::geometry::QueryObject<double>* query_object,
				const drake::geometry::SceneGraphInspector<double>* inspector,
				std::vector<drake::geometry::GeometryId> geometry_ids
			);
}

/* 
 * Convex hull algorithm - Library (C++)
 * 
 * Copyright (c) 2017 Project Nayuki
 * https://www.nayuki.io/page/convex-hull-algorithm
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program (see COPYING.txt and COPYING.LESSER.txt).
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <algorithm>
#include "tools/ConvexHull.h"

using std::vector;


bool Point::operator==(const Point &other) const {
	return x == other.x && y == other.y;
}

bool Point::operator!=(const Point &other) const {
	return x != other.x || y != other.y;
}

bool Point::operator<(const Point &other) const {
	if (x != other.x) return x < other.x;
	else return y < other.y;
}

bool Point::operator>(const Point &other) const {
	if (x != other.x) return x > other.x;
	else return y > other.y;
}

bool Point::operator<=(const Point &other) const {
	if (x != other.x) return x < other.x;
	else return y <= other.y;
}

bool Point::operator>=(const Point &other) const {
	if (x != other.x) return x > other.x;
	else return y >= other.y;
}


std::vector<Eigen::VectorXd> makeConvexHull(const std::vector<Eigen::VectorXd> points)
{
	std::vector<Point> new_points;

	for (int i = 0; i < points.size(); ++i)
	{
		Point p = {
			points[i](0),
			points[i](1),
		};

		new_points.push_back(p);
	}

	std::vector<Point> convex_hull;
	convex_hull = makeConvexHull(new_points);

	std::vector<Eigen::VectorXd> new_convex_hull;
	for (int i = 0; i < convex_hull.size(); ++i)
	{
		Eigen::VectorXd v(2);
		v(0) = convex_hull[i].x;
		v(1) = convex_hull[i].y;

		new_convex_hull.push_back(v);
	}

	return new_convex_hull;
}

vector<Point> makeConvexHull(const vector<Point> &points) {
	vector<Point> newPoints = points;
	std::sort(newPoints.begin(), newPoints.end());
	return makeConvexHullPresorted(newPoints);
}


vector<Point> makeConvexHullPresorted(const vector<Point> &points) {
	if (points.size() <= 1)
		return vector<Point>(points);
	
	// Andrew's monotone chain algorithm. Positive y coordinates correspond to "up"
	// as per the mathematical convention, instead of "down" as per the computer
	// graphics convention. This doesn't affect the correctness of the result.
	
	vector<Point> upperHull;
	for (const Point &p : points) {
		while (upperHull.size() >= 2) {
			const Point &q = *(upperHull.cend() - 1);  // Same as .back()
			const Point &r = *(upperHull.cend() - 2);
			if ((q.x - r.x) * (p.y - r.y) >= (q.y - r.y) * (p.x - r.x))
				upperHull.pop_back();
			else
				break;
		}
		upperHull.push_back(p);
	}
	upperHull.pop_back();
	
	vector<Point> lowerHull;
	for (vector<Point>::const_reverse_iterator it = points.crbegin(); it != points.crend(); ++it) {
		const Point &p = *it;
		while (lowerHull.size() >= 2) {
			const Point &q = *(lowerHull.cend() - 1);  // Same as .back()
			const Point &r = *(lowerHull.cend() - 2);
			if ((q.x - r.x) * (p.y - r.y) >= (q.y - r.y) * (p.x - r.x))
				lowerHull.pop_back();
			else
				break;
		}
		lowerHull.push_back(p);
	}
	lowerHull.pop_back();
	
	if (!(upperHull.size() == 1 && upperHull == lowerHull))
		upperHull.insert(upperHull.end(), lowerHull.cbegin(), lowerHull.cend());
	return upperHull;
}

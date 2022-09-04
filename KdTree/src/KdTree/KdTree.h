#pragma once

using std::vector;


namespace Kdtree {
	const inline double DOUBLE_INFINITY = std::numeric_limits<double>::infinity();

	enum class SPLIT_AXIS {
		AXIS_X = 0,
		AXIS_Y = 1,
		AXIS_Z = 2
	};

	class Point
	{
	public:
		Point() : e{ 0.0, 0.0, 0.0 } {};
		Point(double x, double y) : e{ x, y, 0.0 } {};
		Point(double x, double y, double z) : e{ x, y, z } {};
		Point(const Point& p) : e{ p.e[0], p.e[1], p.e[2] } {};

		Point& operator= (const Point& p) {
			if (this == &p) { return *this; }
			e[0] = p.e[0]; e[1] = p.e[1]; e[2] = p.e[2];
			return *this;
		};

		double operator[] (int i) const {
			if (i < 0 || i > 3) { std::cout << "Array out of index" << std::endl; return DOUBLE_INFINITY; }
			return e[i];
		};

		// double x() const { return e[0]; }
		// double y() const { return e[1]; }
		// double z() const { return e[2]; }

		double GetDistanceToAPoint(const Point& p) const {
			double dx = x - p.x; double dy = y - p.y; double dz = z - p.z;
			return dx * dx + dy * dy + dz * dz;
		}

	public: // short cut :>
		union {
			double e[3];
			struct { double x, y, z; };
		};
	};

	class AABB
	{
	public:
		AABB() : m_bln({ DOUBLE_INFINITY, DOUBLE_INFINITY, DOUBLE_INFINITY }),
			m_trf({ -DOUBLE_INFINITY, -DOUBLE_INFINITY, -DOUBLE_INFINITY }) {}
		AABB(double x0, double x1, double y0, double y1) : m_bln({ x0, y0 }), m_trf({ x1, y1 }) {}
		AABB(double x0, double x1, double y0, double y1, double z0, double z1) : m_bln({ x0, y0, z0 }), m_trf({ x1, y1, z1 }) {}
		AABB(const Point& bln, const Point& trf) : m_bln(bln), m_trf(trf) {}
		AABB(const AABB& aabb) : m_bln(aabb.m_bln), m_trf(aabb.m_trf) {}

		AABB& operator= (const AABB& aabb) {
			// Check self-assignment.
			if (this == &aabb) { return *this; }
			m_bln = aabb.m_bln; m_trf = aabb.m_trf;
			return *this;
		}

		// Point get_bln() const { return m_bln; }
		// Point get_trf() const { return m_trf; }

		bool Intersect(AABB* aabb) const {
			// if (aabb.m_bln[0] > m_trf[0] || aabb.m_bln[1] > m_trf[1] || aabb.m_bln[2] > m_trf[2]) { return false; }
			if (aabb->m_bln[0] >= m_trf[0] || aabb->m_bln[1] >= m_trf[1] || aabb->m_bln[2] >= m_trf[2]) { return false; }
			if (aabb->m_trf[0] < m_bln[0] || aabb->m_trf[1] < m_bln[1] || aabb->m_trf[2] < m_bln[2]) { return false; }
			return true;
		}
		bool Contain(AABB* aabb) const {
			// if (aabb.m_bln[0] < m_bln[0] || aabb.m_bln[1] < m_bln[1] || aabb.m_bln[2] < m_bln[2]) { return false; }
			if (aabb->m_bln[0] <= m_bln[0] || aabb->m_bln[1] <= m_bln[1] || aabb->m_bln[2] <= m_bln[2]) { return false; }
			else if (aabb->m_trf[0] > m_trf[0] || aabb->m_trf[1] > m_trf[1] || aabb->m_trf[2] > m_trf[2]) { return false; }
			else { return true; }
		}
		bool Contain(Point& p) const {
			// if (p[0] < m_bln[0] || p[1] < m_bln[1] || p[2] < m_bln[2]) { return false; }
			if (p[0] <= m_bln[0] || p[1] <= m_bln[1] || p[2] <= m_bln[2]) { return false; }
			else if (p[0] > m_trf[0] || p[1] > m_trf[1] || p[2] > m_trf[2]) { return false; }
			else { return true; }
		}

		double GetDistanceToAPoint(const Point& p) const {  // Nice implement.
			// For dist1 and dis2, assume that dist1 > dist2, ithere are 3 circumstance for them:
			// 1) dist1 * dist2 > 0:
			// 1.1) dist1 > dist2 > 0, the distance is the smaller one, dist2;
			// 1.2) 0 > dist1 > dist2, the distance is the negative bigger one, -dist1;
			// 2) dist1 * dist2 < 0: the distance is 0.
			double dx = (m_bln.x - p.x > 0 ? m_bln.x - p.x : (m_trf.x - p.x < 0 ? p.x - m_bln.x : 0));
			double dy = (m_bln.y - p.y > 0 ? m_bln.y - p.y : (m_trf.y - p.y < 0 ? p.y - m_bln.y : 0));
			double dz = (m_bln.z - p.z > 0 ? m_bln.z - p.z : (m_trf.z - p.z < 0 ? p.z - m_bln.z : 0));
			return dx * dx + dy * dy + dz * dz;
		}

	public:  // short cut :>
		Point m_bln;  // [min_x, min_y, min_z]
		Point m_trf;  // [max_x, max_y, max_z]
	};

	class KdNode {
	public:
		KdNode() : lc(nullptr), rc(nullptr), aabb(nullptr), idxLeft(-1), idxRight(-1) {}
		KdNode(int idx) : lc(nullptr), rc(nullptr), aabb(nullptr), idxLeft(idx), idxRight(idx + 1) {  // Leaf node.
			// std::cout << "Ctor of the leaf node." << std::endl;
		}
		KdNode(int left, int right) : lc(nullptr), rc(nullptr), aabb(nullptr), idxLeft(left), idxRight(right) {  // Subtree node.
			// std::cout << "Ctor of the subtree node." << std::endl;
		}

		bool IsLeaf() const { return lc == nullptr && rc == nullptr; };

	public:  // short cut :>
		KdNode* lc;
		KdNode* rc;
		SPLIT_AXIS splitAxis = SPLIT_AXIS::AXIS_X;  // Default X
		double splitLine;

		AABB* aabb;      // Remember to overrite the copy assignment of AABB
		int idxLeft;    // [idxLeft
		int idxRight;  // idxRight)
	};

	class KdTree {
	public:
		KdTree(const vector<Point>& vecPts);
		KdTree(const vector<Point>& vecPts, int left, int right);

		KdNode* BuildKdTree(AABB* aabb, int left, int right, int depth);

		void RangeSearchFromRoot(AABB* R, vector<int>& ans) const;
		void RangeSearch(KdNode* root, AABB* R, vector<int>& ans) const;

		void KnnSearchFromRoot(int idxSearchPoint, int k, vector<int>& ans) const;
		void KnnSearch(KdNode* root, int idxSearchPoint, int k, std::priority_queue<std::pair<double, int>> pq) const;

		// void RadiusSearchFromRoot(Point& p, double radius, vector<int>& ans) const;
		// void RadiusSearchFromRoot(int idxSearchPoint, double radius, vector<int>& ans) const;
		// void RadiusSearch(KdNode* root, Point& p, double radius, vector<int>& ans) const;
		// void RadiusSearch(KdNode* root, int idxSearchPoint, double radius, vector<int>& ans) const;

		// KdNode* GetRoot() const { return p_root; }
		// vector<int> GetIndices() const { return m_indices; }
		// const vector<Point>* GetVectorPoints() const { return p_vecPts; }

	private:
		KdNode* CreateLeaf(int idx);
		KdNode* CreateSubtree(AABB* aabb, int left, int right);
		double FindMedian(KdNode* root, int idxMid);
		void Divide(KdNode* root, AABB* aabbLeft, AABB* aabbRight);

		bool Inside(KdNode* v, AABB* aabb) const;
		void Report(KdNode* leaf, vector<int>& ans) const;
		void ReportSubtree(KdNode* subtree, vector<int>& ans) const;

	public:  // short cut :>
		KdNode* p_root;
		vector<int> m_indices;
		const vector<Point>* p_vecPts;  // Let the user hold data.
	};
};

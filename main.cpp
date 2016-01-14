#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <algorithm>

#include <cstdio>

#include <boost/algorithm/string.hpp>

using boost::algorithm::trim_left;

class Mesh{

	struct Vertex{
		float x,y,z;
		Vertex(){
			x = y = z = 0.0f;
		}

		Vertex(float x, float y, float z){
			this->x = x;
			this->y = y;
			this->z = z;
		}
	};
	
	struct Facet{
		int a,b,c;
		Facet(){
			a = b = c = -1;
		}
		Facet(int a, int b, int c){
			this->a = a;
			this->b = b;
			this->c = c;
		}
	};

	public:

	std::vector<Vertex> vertices;
	std::vector<Facet> facets;

	Mesh(const std::string &filename){
		/*  Parses an ASCII format STL file and loads the data 
		 *  Data is represented using a double edge data structure
		 * 	Supported operations:
		 * 	-> For each facet, retreive the indices to it's vertices
		 * 		in the vertex list
		 */
		
		std::ifstream input;
		input.open(filename);
		
		//Parse header
		std::string line;
		if(std::getline(input,line)){
			if(!line.compare(0,9,std::string("solid STL"))){
				/* Look for keyword "facet normal"
				 * Dont read the normal value
				 */

				enum parserState {SEEKING_FACET,SEEKING_LOOP,SEEKING_VERTEX,SEEKING_ENDFACET,ERROR};

				parserState ps = SEEKING_FACET;
				int linesParsed = 1, vertex_count = 0, active_vertex_index = 0;
				int active_triplet[3] = {0};
				std::unordered_map<std::string,int> vertex_indices;
				std::string garbage, X, Y, Z;
				Vertex temp;

				std::getline(input,line);	
				trim_left(line);
				while(line.compare(0,8,std::string("endsolid"))){
					linesParsed++;
					switch(ps){
						case SEEKING_FACET:
							if(!line.compare(0,5,std::string("facet"))){// found beginning of facet segment
								ps = SEEKING_LOOP;
							}
							else{
								ps = ERROR;
							}
							break;
						case SEEKING_LOOP:
							if(!line.compare(0,10,std::string("outer loop"))){// found loop declaration
								ps = SEEKING_VERTEX;
								vertex_count = 0;
							}
							else{
								ps = ERROR;
							}
							break;
						case SEEKING_VERTEX:
							if(!line.compare(0,6,std::string("vertex"))){
								std::stringstream ss(line);
								if(vertex_indices.count(line)==0){// vertex not found in history
									ss >> garbage;
									ss >> temp.x >> temp.y >> temp.z;
									vertices.push_back(temp);
									vertex_indices[line] = active_vertex_index++;
								}
								active_triplet[vertex_count++] = vertex_indices[line];
							}
							else if(!line.compare(0,7,std::string("endloop"))){
								if(vertex_count == 3){
									vertex_count = 0;
									facets.push_back(Facet(active_triplet[0],active_triplet[1],active_triplet[2]));
									ps = SEEKING_ENDFACET;
								}
								else{
									ps = ERROR;
								}
							}
							break;
						case SEEKING_ENDFACET:
							if(!line.compare(0,8,std::string("endfacet"))){
								ps = SEEKING_FACET;
							}
							else{
								ps = ERROR;
							}
							break;
						default:
							std::cerr << "Malformed ASCII STL file. Parsed " << linesParsed << " lines before error." << std::endl;
							exit(-1);		
						}
						std::getline(input,line);
						trim_left(line);
					}
				}
			else{
				std::cerr << "Header malformed or file not in ASCII STL format" << std::endl;
				exit(-1);
			}
		}
		else{
			std::cerr << "Parsing error, could not read or open file" << std::endl;
			exit(-1);
		}	
		input.close();
	}

	struct VertexChain{
		std::vector<Vertex> nodes;
		std::vector<std::pair<int,int> > edges;
	};

	private:

	inline Vertex& RV(const int idx){
		return this->vertices[idx];
	}

	inline Facet& RF(const int idx){
		return this->facets[idx];
	}

	public:

	enum eventType {LINE_BEGIN,LINE_END};
	enum intersectionType {LOWER,UPPER};


	struct Intersection{
		int a,b,c,d;
		intersectionType i = LOWER;

		Intersection(){
			a=b=c=d=0;
		}

		Intersection(int a,int b,int c, int d, intersectionType i){
			this->a=a;
			this->b=b;
			this->c=c;
			this->d=d;
			this->i=i;
		}
	};

	struct sweepLineEvent{
		int facetid;
		Intersection intersection;
		eventType type;
		float eventZ;
		sweepLineEvent(int facetid,eventType t,float eventZ,Intersection intersection){
			this -> facetid = facetid;
			this -> type = t;
			this -> eventZ = eventZ;
			this -> intersection = intersection;
		}

		inline static bool zCompare(const sweepLineEvent &a, const sweepLineEvent &b){
			return a.eventZ < b.eventZ;
		}

		inline static bool typeCompare(const sweepLineEvent &a, const sweepLineEvent &b){
			return a.type < b.type;
		}
	};

	long long int computeId(const sweepLineEvent &in){
		return (in.facetid<<1) + in.intersection.i;
	}

	void printVertex(const Vertex &v){
		std::cout << "(" << v.x << "," << v.y << "," << v.z << ")";
	}

	std::tuple<float,float,float,float> intersect2(const Intersection &i, const float z){
		const auto A = RV(i.a);
		const auto B = RV(i.b);
		const auto C = RV(i.c);
		const auto D = RV(i.d);

		//float rx1 = x_min + (x_max-x_min)/(z_max-z_min)*z;
		//float ry1 = y_min + (y_max-y_min)/(z_max-z_min)*z;
	
		float z_min = A.z;
		float z_max = B.z;

		float rx1 = A.x + (A.x-B.x)/(A.z-B.z)*(z-z_min);
		float ry1 = A.y + (A.y-B.y)/(A.z-B.z)*(z-z_min);
		

		//float rx2 = x_min + (x_max-x_min)/(z_max-z_min)*z;
		//float ry2 = y_min + (y_max-y_min)/(z_max-z_min)*z;

		z_min = C.z;
		z_max = D.z;
		
		float rx2 = C.x + (C.x-D.x)/(C.z-D.z)*(z-z_min);
		float ry2 = C.y + (C.y-D.y)/(C.z-D.z)*(z-z_min);

		return std::tuple<float,float,float,float>(rx1,ry1,rx2,ry2);
	}

	void doSweepLine(float zMin = 0,float zMax = 100.0, int no = 100){
		/* Construct a queue of sweepLine events
		   Every vertex is an event
		   zMin vertices are LINE_BEGIN events
		   zMax vertices are LINE_END events
		   zMid vertices are LINE_END followed by LINE_BEGIN (LINE_SWITCH) events
		*/

		std::vector<sweepLineEvent> sweepQueue;
		int facetid = 0;
		for(const auto &f : facets){

		//	std::cout << f.a << " " << RV(f.a).x << " " << RV(f.a).y << " " << RV(f.a).z << std::endl;
		//	std::cout << f.b << " " << RV(f.b).x << " " << RV(f.b).y << " " << RV(f.b).z << std::endl;
		//	std::cout << f.c << " " << RV(f.c).x << " " << RV(f.c).y << " " << RV(f.c).z << std::endl;

			int minZVertex = f.a, midZVertex = f.b, maxZVertex = f.a;	
			minZVertex = RV(minZVertex).z<=RV(f.b).z?minZVertex:f.b;
			minZVertex = RV(minZVertex).z<=RV(f.c).z?minZVertex:f.c;
			maxZVertex = RV(maxZVertex).z>RV(f.b).z?maxZVertex:f.b;
			maxZVertex = RV(maxZVertex).z>RV(f.c).z?maxZVertex:f.c;
			if(minZVertex == maxZVertex){
				midZVertex = minZVertex;	
			}
			else if((minZVertex == f.a && maxZVertex == f.c) || (minZVertex == f.c && maxZVertex == f.a)){
				midZVertex = f.b;
			}
			else if((minZVertex == f.b && maxZVertex == f.c) || (minZVertex == f.c && maxZVertex == f.b)){
				midZVertex = f.a;
			}
			else if((minZVertex == f.a && maxZVertex == f.b) || (minZVertex == f.b && maxZVertex == f.a)){
				midZVertex = f.c;
			}
		//	std::cout << "minZ " << minZVertex << " midZ " << midZVertex << " maxZ " << maxZVertex << std::endl << std::endl;;
			sweepQueue.push_back(sweepLineEvent(facetid,LINE_BEGIN,RV(minZVertex).z,Intersection(minZVertex,maxZVertex,minZVertex,midZVertex,LOWER)));
			sweepQueue.push_back(sweepLineEvent(facetid,LINE_END,RV(midZVertex).z,Intersection(minZVertex,maxZVertex,minZVertex,midZVertex,LOWER)));
			sweepQueue.push_back(sweepLineEvent(facetid,LINE_BEGIN,RV(midZVertex).z,Intersection(midZVertex,maxZVertex,minZVertex,maxZVertex,UPPER)));
			sweepQueue.push_back(sweepLineEvent(facetid++,LINE_END,RV(maxZVertex).z,Intersection(midZVertex,maxZVertex,minZVertex,maxZVertex,UPPER)));
		}
		// zSort all vertices and build events
		std::stable_sort(sweepQueue.begin(),sweepQueue.end(),sweepLineEvent::typeCompare);
		std::stable_sort(sweepQueue.begin(),sweepQueue.end(),sweepLineEvent::zCompare);
		//std::cout << "BUFFER SIZE " << sweepQueue.size() << std::endl;
		
		//for(const auto i:sweepQueue){
		//	std::cout << (i.type?"LINE_END ":"LINE_BEGIN ") << computeId(i) << " " << i.eventZ  << std::endl;
		//}
		
		//getchar();

		// Move sweep line upwards in regular increments and process all events
		float step = (zMax - zMin)/no;
		float currentZ = zMin;

		auto queuePosition = sweepQueue.begin();
		std::unordered_map<long int,Intersection> current_segments;
		for(int i = 0; i<no; i++){
			while(queuePosition->eventZ < currentZ){
				// Process event
				if(queuePosition == sweepQueue.end())
					break;
				if(queuePosition->type == LINE_BEGIN){
					//std::cout << "inserted " << computeId(*queuePosition) << std::endl;
					current_segments[computeId(*queuePosition)] = queuePosition->intersection;
				}
				else if(queuePosition->type == LINE_END){
					if(current_segments.count(computeId(*queuePosition))){
						//std::cout << "removed " << computeId(*queuePosition) << std::endl;
						current_segments.erase(computeId(*queuePosition));
					}
					else{
						// Should raise error but am ignoring
					}
				}
				else{
					std::cerr << "Malformed event code: Fatal exception" << std::endl;
					exit(-1);
				}
				++queuePosition;
			}
			// std::cout << " z => " << currentZ  << ", Polygon size = " << current_segments.size() << std::endl;
			if(current_segments.size()>0){
				//std::cout << "------" << std::endl;
				for(const auto seg:current_segments){
				//	std::cout << seg.second.a << "," << seg.second.b << "," << seg.second.c << "," << seg.second.d << std::endl;
					const auto it = intersect2(seg.second,currentZ);
					std::cout << std::get<0>(it) << "," << std::get<1>(it) << ","  <<currentZ  << "\n" << std::get<2>(it) << "," << std::get<3>(it)<< "," << currentZ  << std::endl;
				}
			//	std::cout << "------ \n" << std::endl;
			}
			else{
				//std::cout << "No lines" << std::endl;
			}
			currentZ += step;
		}
	}
};

int main(){
	Mesh mesh("sphere.stl");
	mesh.doSweepLine(-10,10,200);
}

#include "wolf_exploration/artifacts_search.h"


int main()
{

std::vector<geometry_msgs::Point> centroids, centroids_original;

geometry_msgs::Point p;

p.x = 0.0;
p.y = 0.0;
p.z = 0.0;
centroids.push_back(p);

p.x = 0.1;
p.y = 0.0;
p.z = 0.0;
centroids.push_back(p);

p.x = 0.0;
p.y = 0.2;
p.z = 0.0;
centroids.push_back(p);

p.x = 0.0;
p.y = 0.4;
p.z = 0.0;
centroids.push_back(p);

p.x = 0.1;
p.y = 0.2;
p.z = 0.0;
centroids.push_back(p);

p.x = 0.1;
p.y = 0.3;
p.z = 0.0;
centroids.push_back(p);


centroids_original = centroids;


// Merge the centroids
std::cout << "Size before: " << centroids.size() << std::endl;
if(centroids.size() > 2)
{
  std::vector<geometry_msgs::Point>::iterator it_main, it_comp;
  geometry_msgs::Point tmp_point;
  it_main = centroids.begin();
  while (it_main != centroids.end())
  {
    it_comp = it_main+1;

    std::cout << "it_main " << " x " <<  it_main->x << " y " << it_main->y << std::endl;
    while (it_comp != centroids.end())
    {
      std::cout << "it_comp " << " x " <<  it_comp->x << " y " << it_comp->y << std::endl;

      double x = (it_main->x - it_comp->x);
      double y = (it_main->y - it_comp->y);
      if(std::sqrt( x*x + y*y ) <= 0.1)
      {
        tmp_point.x = (it_main->x + it_comp->x)/2.0;
        tmp_point.y = (it_main->y + it_comp->y)/2.0;

        *it_main = tmp_point;

        it_comp = centroids.erase(it_comp);

        std::cout << "Merged!" << std::endl;
      }
      else {
        ++it_comp;
      }
    }
    ++it_main;
  }
}

std::cout << "Size after: " << centroids.size() << std::endl;

for(unsigned int i = 0;i<centroids.size(); i++)
  std::cout << "Centroid["<<i<<"] " << " x " << centroids[i].x << " y " << centroids[i].y << std::endl;

return 0;

}

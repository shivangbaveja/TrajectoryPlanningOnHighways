#include<cmath>
#include<boost/python/module.hpp>
#include<boost/python/def.hpp>
#include<boost/python/extract.hpp>
#include<boost/python/numpy.hpp>

using namespace boost::python;
namespace np = boost::python::numpy;

/* Define a C++ function as you would */
np::ndarray eucnorm(np::ndarray axis){
    return axis;
}

/* Define your module name within BOOST_PYTHON_MODULE */ 
BOOST_PYTHON_MODULE(vectors){ 
  /* Initialise numpy */
  np::initialize();
  /* Define your function, named eucnorm */
  def("eucnorm", eucnorm);
}


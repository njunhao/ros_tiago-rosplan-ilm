#include "rosplan_interface_search/RPSearch.h"

// Put all definitions in RPSearch.h as this is a class template
// Otherwise, linker will return unresolved methods errors. This is because
// class templates are not instantiated without knowing the typename. Hence,
// when a derived class based on a class template is instantiated, we get unresolved errors.
#include "Filter.h"

filter_kind iq_filter={

.data=0,
.last_data=0,	
.alpha=0.85f,

};
filter_kind id_filter={

.data=0,
.last_data=0,	
.alpha=0.85f,

};
filter_kind SMO_VAlpha_filter={

.data=0,
.last_data=0,	
.alpha=0.85f,

};
filter_kind SMO_VBeta_filter={

.data=0,
.last_data=0,	
.alpha=0.85f,

};
filter_kind Active_flux_VAlpha_filter={

.data=0,
.last_data=0,	
.alpha=0.15f,

};
filter_kind Active_flux_VBeta_filter={

.data=0,
.last_data=0,	
.alpha=0.15f,

};
filter_kind Nonlinear_flux_VAlpha_filter={

.data=0,
.last_data=0,	
.alpha= 0.05f,

};
filter_kind Nonlinear_flux_VBeta_filter={

.data=0,
.last_data=0,	
.alpha= 0.05f,

};
filter_kind Luenberger_VAlpha_filter={

.data=0,
.last_data=0,	
.alpha= 0.7f,

};
filter_kind Luenberger_VBeta_filter={

.data=0,
.last_data=0,	
.alpha= 0.7f,

};
filter_kind we_fore_filter={

.data=0,
.last_data=0,	
.alpha=0.1f,

};
	float Filter_func(filter_kind * filter,float sth )
{
  filter->data = filter->alpha * sth + (1-filter->alpha) * filter->last_data;
	filter->last_data = filter->data;
	return filter->data;
}

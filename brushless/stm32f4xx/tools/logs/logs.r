#!/usr/bin/env Rscript

# Copyright (C) 2019  Adrien Boussicault <adrien.boussicault@labri.fr>
#
# This program is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either
# version 3 of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this program.  If not, see
# <http://www.gnu.org/licenses/>.

# To install, just start R by typing 
# R
# and then execute  
# install.packages("PACKAGE_NAME", dpendencies=TRUE)
#library(tidyverse)
library(ggplot2)
library(scales)


make_image <- function(data, name, display) {
  data <- reshape(
    data, idvar="cpt", varying=plotnames, v.name=c("value"), timevar="column", times=plotnames,
    direction="long" 
  )
  
  data$column <- factor(data$column,  levels=plotnames_order)

  #print( head(data) )
  g <- ggplot(data, aes(x=t, y=value, group=column))
  g <- g+geom_line();
  g <- g+geom_point(color='red', size=0.05 ); #, shape="x");
  g <- g + facet_grid(column ~ ., scales="free_y"); #rows = vars(t));
  g <- g + theme_bw() + theme(axis.text.x = element_text(angle = 90));
  g <- g+scale_x_continuous(breaks=scales::pretty_breaks(n=20))
  ggsave( paste0(dirname, "/", name, ".png") );
  if( display ){
    browseURL(paste0(dirname, "/", name, ".png"));
    #library(tcltk);
    #X11();
    #plot(g);
    #prompt <- "hit space bar";
  }
}

cut_data <- function(data, t1, t2) {
  res <- data[which(data$t >= t1),]
  res <- res[which(res$t <= t2),]
  return(res)
}


make_cuted_image <- function(data, name, t1, t2, display) {
  data1 <- cut_data(data, t1, t2)
  make_image( data1, paste0(name, "_", toString(t1) , "_", toString(t2), "_"), display )
}


default_post_process <- function(data, N, frequence, t1, t2) {
  #data$U <- data$pwm_u-(data$pwm_u+data$pwm_v+data$pwm_w)/3.0
  #data$V <- data$pwm_v-(data$pwm_u+data$pwm_v+data$pwm_w)/3.0
  #data$W <- data$pwm_w-(data$pwm_u+data$pwm_v+data$pwm_w)/3.0

  #data$theo_i_all <- ( abs(data$U) + abs(data$V) + abs(data$W) )/2.0

  return(data)
}

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
library("optparse")
library(tools)

option_list = list(
  make_option(
    c("-d", "--data_file"),
    type="character", default=NULL, help="dataset file path", 
    metavar="FILE_PATH"
  ),
	make_option(
    c("-f", "--frequence"),
    type="double", default=10000.0, 
    help="Sample frequence [default= %default]", metavar="FREQUENCE"
  ),
	make_option(
    c("-v", "--over_sample"),
    type="integer", default=1, 
    help="Sample have been oversampled by a N factor [default= %default]", metavar="N"
  ),
	make_option(
    c("-s", "--starting_time"),
    type="character", default=NA, 
    help="stating time in seconds [default= %default]", metavar="TIME"
  ),
	make_option(
    c("-e", "--ending_time"),
    type="double", default=NA, 
    help="stating time in seconds [default= %default]", metavar="TIME"
  ),
	make_option(
    c("-p", "--period"),
    type="double", default=NA, 
    help=paste0(
      "compute ending_time or starting_time according to the relation : ",
      "ending_time - starting_time = period [default= %default]"
    ), metavar="TIME"
  ),
	make_option(
    c("-c", "--cut"), action="store_true", default=FALSE,
     help="Split into multiple image using period, and starting and ending time"
  ),
	make_option(
    c("-q", "--quiet_display"), action="store_true", default=FALSE,
     help="Do not display the image"
  )
); 
opt_parser = OptionParser(option_list=option_list);
opt = parse_args(opt_parser);

if (is.null(opt$data_file)){
  print_help(opt_parser);
  stop("At least one argument must be supplied (input file).n", call.=FALSE);
}else{
  basename <- basename(opt$data_file);
  dirname <- dirname(opt$data_file);
  name <- file_path_sans_ext(basename);
}

if (
  ! is.na(opt$ending_time)  && 
  ! is.na(opt$starting_time)  && 
  ! is.na(opt$period) &&
  ! opt$cut
){
  print_help(opt_parser);
  stop(
    "Do not define at the same time ending_time, starting_time and period",
    call.=FALSE
  );
}
if (
  is.na(opt$ending_time)  && 
  is.na(opt$starting_time)  && 
  ! is.na(opt$period)
){
  print_help(opt_parser);
  stop(
    "Period should be defined with ending_time or starting_time.",
    call.=FALSE
  );
}

save_png <- opt$output;
N <- opt$over_sample;
frequence <- as.double( opt$frequence )

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

data <- read.csv( paste0(dirname, "/", name, ".csv") )

data$U <- data$pwm_u-(data$pwm_u+data$pwm_v+data$pwm_w)/3.0
data$V <- data$pwm_v-(data$pwm_u+data$pwm_v+data$pwm_w)/3.0
data$W <- data$pwm_w-(data$pwm_u+data$pwm_v+data$pwm_w)/3.0

data$theo_i_all <- ( abs(data$U) + abs(data$V) + abs(data$W) )/2.0

plotnames <- names(data)[2:ncol(data)]
plotnames_order <- plotnames

data$t <- (data$cpt - min(data$cpt))*N*(1.0/frequence)

max_t <- max(data$t)
min_t <- min(data$t)

t1 <- min_t;
if ( ! is.na(opt$starting_time) ){
  t1 <- as.double(opt$starting_time);
  if( t1 < min_t || t1 > max_t ){
    stop(
      paste(
        "starting time ", toString(t1), "should be between be beetwen", 
        toString(min_t) , "and", toString(max_t), "."
      ), call.=FALSE 
    );
  }
}else{
  if( ! opt$cut && ! is.na(opt$period) ){
    t1 <- ( as.double(opt$ending_time) - as.double(opt$period) );
  }
}

t2 <- max_t;
if ( ! is.na(opt$ending_time) ){
  t2 <- as.double(opt$ending_time);
  if( t2 < min_t || t2 > max_t ){
    stop(
      paste(
        "ending time ", toString(t2), "should be between be beetwen", 
        toString(min_t) , "and", toString(max_t), "."
      ) , call.=FALSE 
    );
  }
}else{
  if( ! opt$cut && ! is.na(opt$period) ){
    t2 <- ( as.double(opt$starting_time) + as.double(opt$period) );
  }
}
if( t1 > t2 ){
    stop( "ending time should be greater than starting time.", call.=FALSE );
}

display = !opt$quiet_display
if( opt$cut ){
  period = as.double(opt$period);
  t <- t1;
  counter = 0;
  while( t - t2  < -0.5/frequence ){
    make_cuted_image(
      data, paste0( toString(counter, width=4), "_", name),
      t, min(t+period,t2), display
    );
    display = FALSE;
    t <- t + period;
    counter <- counter + 1
  }
}else{
  if ( is.na(opt$starting_time) && is.na(opt$ending_time)){
    make_image( data, name, display )
  }else{
    make_cuted_image( data, name, t1, t2, display )
  }
}



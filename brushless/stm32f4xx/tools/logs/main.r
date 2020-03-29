#!/usr/bin/env Rscript

library(optparse)
library(tools)
library(scriptName)

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
    c("-x", "--post_process"), type="character", default=NULL,
     help="Post process data with a specific  scipt [default= %default]", 
    metavar="R_SCRIPT"
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

source_dir = dirname( current_filename() )
source( paste0(source_dir, '/logs.r') )

save_png <- opt$output;
N <- opt$over_sample;
frequence <- as.double( opt$frequence )

data <- read.csv( paste0(dirname, "/", name, ".csv") )
minval = min(data$cpt)
data$time <- (data$cpt - minval)*N*(1.0/frequence)

max_t <- max(data$time)
min_t <- min(data$time)

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

default_post_process <- function(data, N, frequence, t1, t2) {
  return(data)
}

if( is.null( opt$post_process ) ){
  local_post_process_path <- paste0(dirname, "/", name , ".r")
  if( file.exists( local_post_process_path ) ){
    source(local_post_process_path)
    data <- post_process(data, N, frequencei, t1, t2) 
  }else{
    data <- default_post_process(data, N, frequence, t1, t2);
  }
}else{
  source(opt$post_process)
  data <- post_process(data, N, frequence, t1, t2) 
}
plotnames <- names(data)[2:ncol(data)]
plotnames_order <- plotnames

data$t <- data$time


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

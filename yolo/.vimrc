call plug#begin('~/.vim/plugged')                                                        
	Plug 'junegunn/vim-plug'                                                                                                                       
	Plug 'scrooloose/nerdtree'                                                               
	Plug 'vim-scripts/taglist.vim'                                              
call plug#end() 
"->NERDTree目录树插件---配置选项=====================================================         
let g:NERDTreeDirArrowExpandable = '▸'  "目录图标                                                                
let g:NERDTreeDirArrowCollapsible = '▾'
"autocmd vimenter * NERDTree                "自动打开目录树
"vim【无文件】也显示目录树 
autocmd StdinReadPre * let s:std_in=1
autocmd VimEnter * if argc() == 0 && !exists("s:std_in") | NERDTree | endif
"vim打开目录文件也显示目录树？
autocmd StdinReadPre * let s:std_in=1 
autocmd VimEnter * if argc() == 1 && isdirectory(argv()[0]) && !exists("s:std_in") | exe 'NERDTree' argv()[0] | wincmd p | ene     | endif
"CRTL+N开关目录树
map <C-n> :NERDTreeToggle<CR>
"关闭最后一个文件，同时关闭目录树
autocmd bufenter * if (winnr("$") == 1 && exists("b:NERDTree") && b:NERDTree.isTabTree()) | q | endif
"<-NERDTree目录树插件---配置选项===============================================================
"->taglist浏览插件配置=========================================     
"taglist窗口显示在右侧，缺省为左侧     
let Tlist_Use_Right_Window=1    
"设置ctags路径"将taglist与ctags关联     
let Tlist_Ctags_Cmd = '/usr/bin/ctags'     
"启动vim后自动打开taglist窗口     
let Tlist_Auto_Open = 1     
"不同时显示多个文件的tag，只显示当前文件的     
"不同时显示多个文件的tag，仅显示一个     
let Tlist_Show_One_File = 1     
"taglist为最后一个窗口时，退出vim     
let Tlist_Exit_OnlyWindow = 1     
"let Tlist_Use_Right_Window =1     
"设置taglist窗口大小     
"let Tlist_WinHeight = 100     
"let Tlist_WinWidth = 40     
"设置taglist打开关闭的快捷键F8     
noremap <F8> :TlistToggle<CR>     
"更新ctags标签文件快捷键设置     
noremap <F6> :!ctags -R<CR>     
"<-taglist=========================================  

use crate::Sealed;

use super::*;
use crate::pac::adc1;
#[cfg(not(feature = "stm32f100"))]
use crate::pac::adc2;
#[cfg(not(feature = "stm32f100"))]
use crate::pac::adc3;

pub trait AdcRB: Sealed {
    fn sr(&self) -> &adc1::SR;
    type CR1rs: reg::Cr1R + reg::Cr1W;
    fn cr1(&self) -> &Reg<Self::CR1rs>;
    type CR2rs: reg::Cr2R + reg::Cr2W;
    fn cr2(&self) -> &Reg<Self::CR2rs>;
    fn htr(&self) -> &adc1::HTR;
    fn jdr(&self, n: usize) -> &adc1::JDR;
    fn jdr1(&self) -> &adc1::JDR {
        self.jdr(0)
    }
    fn jdr2(&self) -> &adc1::JDR {
        self.jdr(1)
    }
    fn jdr3(&self) -> &adc1::JDR {
        self.jdr(2)
    }
    fn jdr4(&self) -> &adc1::JDR {
        self.jdr(3)
    }
    fn jofr(&self, n: usize) -> &adc1::JOFR;
    fn jofr1(&self) -> &adc1::JOFR {
        self.jofr(0)
    }
    fn jofr2(&self) -> &adc1::JOFR {
        self.jofr(1)
    }
    fn jofr3(&self) -> &adc1::JOFR {
        self.jofr(2)
    }
    fn jofr4(&self) -> &adc1::JOFR {
        self.jofr(3)
    }
    fn jsqr(&self) -> &adc1::JSQR;
    fn ltr(&self) -> &adc1::LTR;
    fn smpr1(&self) -> &adc1::SMPR1;
    fn smpr2(&self) -> &adc1::SMPR2;
    fn sqr1(&self) -> &adc1::SQR1;
    fn sqr2(&self) -> &adc1::SQR2;
    fn sqr3(&self) -> &adc1::SQR3;
    type DRrs: reg::Dr;
    fn dr(&self) -> &Reg<Self::DRrs>;
}

wrap_r! {
    pub trait Cr1R {
        fn awdch(&self) -> adc1::cr1::AWDCH_R;
        fn eocie(&self) -> adc1::cr1::EOCIE_R;
        fn awdie(&self) -> adc1::cr1::AWDIE_R;
        fn jeocie(&self) -> adc1::cr1::JEOCIE_R;
        fn scan(&self) -> adc1::cr1::SCAN_R;
        fn awdsgl(&self) -> adc1::cr1::AWDSGL_R;
        fn jauto(&self) -> adc1::cr1::JAUTO_R;
        fn discen(&self) -> adc1::cr1::DISCEN_R;
        fn jdiscen(&self) -> adc1::cr1::JDISCEN_R;
        fn discnum(&self) -> adc1::cr1::DISCNUM_R;
        fn jawden(&self) -> adc1::cr1::JAWDEN_R;
        fn awden(&self) -> adc1::cr1::AWDEN_R;
    }
}

wrap_w! {
    pub trait Cr1W {
        fn awdch(&mut self) -> adc1::cr1::AWDCH_W<REG>;
        fn eocie(&mut self) -> adc1::cr1::EOCIE_W<REG>;
        fn awdie(&mut self) -> adc1::cr1::AWDIE_W<REG>;
        fn jeocie(&mut self) -> adc1::cr1::JEOCIE_W<REG>;
        fn scan(&mut self) -> adc1::cr1::SCAN_W<REG>;
        fn awdsgl(&mut self) -> adc1::cr1::AWDSGL_W<REG>;
        fn jauto(&mut self) -> adc1::cr1::JAUTO_W<REG>;
        fn discen(&mut self) -> adc1::cr1::DISCEN_W<REG>;
        fn jdiscen(&mut self) -> adc1::cr1::JDISCEN_W<REG>;
        fn discnum(&mut self) -> adc1::cr1::DISCNUM_W<REG>;
        fn jawden(&mut self) -> adc1::cr1::JAWDEN_W<REG>;
        fn awden(&mut self) -> adc1::cr1::AWDEN_W<REG>;
    }
}

wrap_r! {
    pub trait Cr2R {
        fn adon(&self) -> adc1::cr2::ADON_R;
        fn cont(&self) -> adc1::cr2::CONT_R;
        fn cal(&self) -> adc1::cr2::CAL_R;
        fn rstcal(&self) -> adc1::cr2::RSTCAL_R;
        fn dma(&self) -> adc1::cr2::DMA_R;
        fn align(&self) -> adc1::cr2::ALIGN_R;
        fn jexttrig(&self) -> adc1::cr2::JEXTTRIG_R;
        fn exttrig(&self) -> adc1::cr2::EXTTRIG_R;
        fn jswstart(&self) -> adc1::cr2::JSWSTART_R;
        fn swstart(&self) -> adc1::cr2::SWSTART_R;
        fn tsvrefe(&self) -> adc1::cr2::TSVREFE_R;
    }
}

wrap_w! {
    pub trait Cr2W {
        fn adon(&mut self) -> adc1::cr2::ADON_W<REG>;
        fn cont(&mut self) -> adc1::cr2::CONT_W<REG>;
        fn cal(&mut self) -> adc1::cr2::CAL_W<REG>;
        fn rstcal(&mut self) -> adc1::cr2::RSTCAL_W<REG>;
        fn dma(&mut self) -> adc1::cr2::DMA_W<REG>;
        fn align(&mut self) -> adc1::cr2::ALIGN_W<REG>;
        fn jexttrig(&mut self) -> adc1::cr2::JEXTTRIG_W<REG>;
        fn exttrig(&mut self) -> adc1::cr2::EXTTRIG_W<REG>;
        fn jswstart(&mut self) -> adc1::cr2::JSWSTART_W<REG>;
        fn swstart(&mut self) -> adc1::cr2::SWSTART_W<REG>;
        fn tsvrefe(&mut self) -> adc1::cr2::TSVREFE_W<REG>;
    }
}

pub trait ExtSelW {
    fn select_swstart(&mut self) -> &mut Self;
}

impl<REG: reg::ExtSelW> ExtSelW for W<REG> {
    fn select_swstart(&mut self) -> &mut Self {
        REG::select_swstart(self)
    }
}

wrap_r! {
    pub trait Dr {
        fn data(&self) -> adc1::dr::DATA_R;
    }
}

mod reg {
    use super::*;

    pub trait Cr1R: RegisterSpec<Ux = u32> + Readable + Sized {
        fn awdch(r: &R<Self>) -> adc1::cr1::AWDCH_R;
        fn eocie(r: &R<Self>) -> adc1::cr1::EOCIE_R;
        fn awdie(r: &R<Self>) -> adc1::cr1::AWDIE_R;
        fn jeocie(r: &R<Self>) -> adc1::cr1::JEOCIE_R;
        fn scan(r: &R<Self>) -> adc1::cr1::SCAN_R;
        fn awdsgl(r: &R<Self>) -> adc1::cr1::AWDSGL_R;
        fn jauto(r: &R<Self>) -> adc1::cr1::JAUTO_R;
        fn discen(r: &R<Self>) -> adc1::cr1::DISCEN_R;
        fn jdiscen(r: &R<Self>) -> adc1::cr1::JDISCEN_R;
        fn discnum(r: &R<Self>) -> adc1::cr1::DISCNUM_R;
        fn jawden(r: &R<Self>) -> adc1::cr1::JAWDEN_R;
        fn awden(r: &R<Self>) -> adc1::cr1::AWDEN_R;
    }

    pub trait Cr1W: RegisterSpec<Ux = u32> + Writable + Resettable + Sized {
        fn awdch(w: &mut W<Self>) -> adc1::cr1::AWDCH_W<Self>;
        fn eocie(w: &mut W<Self>) -> adc1::cr1::EOCIE_W<Self>;
        fn awdie(w: &mut W<Self>) -> adc1::cr1::AWDIE_W<Self>;
        fn jeocie(w: &mut W<Self>) -> adc1::cr1::JEOCIE_W<Self>;
        fn scan(w: &mut W<Self>) -> adc1::cr1::SCAN_W<Self>;
        fn awdsgl(w: &mut W<Self>) -> adc1::cr1::AWDSGL_W<Self>;
        fn jauto(w: &mut W<Self>) -> adc1::cr1::JAUTO_W<Self>;
        fn discen(w: &mut W<Self>) -> adc1::cr1::DISCEN_W<Self>;
        fn jdiscen(w: &mut W<Self>) -> adc1::cr1::JDISCEN_W<Self>;
        fn discnum(w: &mut W<Self>) -> adc1::cr1::DISCNUM_W<Self>;
        fn jawden(w: &mut W<Self>) -> adc1::cr1::JAWDEN_W<Self>;
        fn awden(w: &mut W<Self>) -> adc1::cr1::AWDEN_W<Self>;
    }

    pub trait Cr2R: RegisterSpec<Ux = u32> + Readable + Sized {
        fn adon(r: &R<Self>) -> adc1::cr2::ADON_R;
        fn cont(r: &R<Self>) -> adc1::cr2::CONT_R;
        fn cal(r: &R<Self>) -> adc1::cr2::CAL_R;
        fn rstcal(r: &R<Self>) -> adc1::cr2::RSTCAL_R;
        fn dma(r: &R<Self>) -> adc1::cr2::DMA_R;
        fn align(r: &R<Self>) -> adc1::cr2::ALIGN_R;
        fn jexttrig(r: &R<Self>) -> adc1::cr2::JEXTTRIG_R;
        fn exttrig(r: &R<Self>) -> adc1::cr2::EXTTRIG_R;
        fn jswstart(r: &R<Self>) -> adc1::cr2::JSWSTART_R;
        fn swstart(r: &R<Self>) -> adc1::cr2::SWSTART_R;
        fn tsvrefe(r: &R<Self>) -> adc1::cr2::TSVREFE_R;
    }
    pub trait Cr2W: RegisterSpec<Ux = u32> + Writable + Resettable + Sized + ExtSelW {
        fn adon(w: &mut W<Self>) -> adc1::cr2::ADON_W<Self>;
        fn cont(w: &mut W<Self>) -> adc1::cr2::CONT_W<Self>;
        fn cal(w: &mut W<Self>) -> adc1::cr2::CAL_W<Self>;
        fn rstcal(w: &mut W<Self>) -> adc1::cr2::RSTCAL_W<Self>;
        fn dma(w: &mut W<Self>) -> adc1::cr2::DMA_W<Self>;
        fn align(w: &mut W<Self>) -> adc1::cr2::ALIGN_W<Self>;
        fn jexttrig(w: &mut W<Self>) -> adc1::cr2::JEXTTRIG_W<Self>;
        fn exttrig(w: &mut W<Self>) -> adc1::cr2::EXTTRIG_W<Self>;
        fn jswstart(w: &mut W<Self>) -> adc1::cr2::JSWSTART_W<Self>;
        fn swstart(w: &mut W<Self>) -> adc1::cr2::SWSTART_W<Self>;
        fn tsvrefe(w: &mut W<Self>) -> adc1::cr2::TSVREFE_W<Self>;
    }
    pub trait Dr: RegisterSpec<Ux = u32> + Readable + Sized {
        fn data(r: &R<Self>) -> adc1::dr::DATA_R;
    }

    pub trait ExtSelW: RegisterSpec<Ux = u32> + Sized {
        fn select_swstart(w: &mut W<Self>) -> &mut W<Self>;
    }
}

macro_rules! impl_ext {
    ($adc:ident) => {
        impl Sealed for $adc::RegisterBlock {}
        impl AdcRB for $adc::RegisterBlock {
            type CR1rs = $adc::cr1::CR1rs;
            type CR2rs = $adc::cr2::CR2rs;
            type DRrs = $adc::dr::DRrs;
            impl_reg! {
                sr -> &adc1::SR;
                cr1 -> &Reg<Self::CR1rs>;
                cr2 -> &Reg<Self::CR2rs>;
                htr -> &adc1::HTR;
                jdr: n -> &adc1::JDR;
                jofr: n -> &adc1::JOFR;
                jsqr -> &adc1::JSQR;
                ltr -> &adc1::LTR;
                smpr1 -> &adc1::SMPR1;
                smpr2 -> &adc1::SMPR2;
                sqr1 -> &adc1::SQR1;
                sqr2 -> &adc1::SQR2;
                sqr3 -> &adc1::SQR3;
                dr -> &Reg<Self::DRrs>;
            }
        }

        impl reg::Cr1R for $adc::cr1::CR1rs {
            impl_read! {
                awdch -> adc1::cr1::AWDCH_R;
                eocie -> adc1::cr1::EOCIE_R;
                awdie -> adc1::cr1::AWDIE_R;
                jeocie -> adc1::cr1::JEOCIE_R;
                scan -> adc1::cr1::SCAN_R;
                awdsgl -> adc1::cr1::AWDSGL_R;
                jauto -> adc1::cr1::JAUTO_R;
                discen -> adc1::cr1::DISCEN_R;
                jdiscen -> adc1::cr1::JDISCEN_R;
                discnum -> adc1::cr1::DISCNUM_R;
                jawden -> adc1::cr1::JAWDEN_R;
                awden -> adc1::cr1::AWDEN_R;
            }
        }
        impl reg::Cr1W for $adc::cr1::CR1rs {
            impl_write! {
                awdch -> adc1::cr1::AWDCH_W<Self>;
                eocie -> adc1::cr1::EOCIE_W<Self>;
                awdie -> adc1::cr1::AWDIE_W<Self>;
                jeocie -> adc1::cr1::JEOCIE_W<Self>;
                scan -> adc1::cr1::SCAN_W<Self>;
                awdsgl -> adc1::cr1::AWDSGL_W<Self>;
                jauto -> adc1::cr1::JAUTO_W<Self>;
                discen -> adc1::cr1::DISCEN_W<Self>;
                jdiscen -> adc1::cr1::JDISCEN_W<Self>;
                discnum -> adc1::cr1::DISCNUM_W<Self>;
                jawden -> adc1::cr1::JAWDEN_W<Self>;
                awden -> adc1::cr1::AWDEN_W<Self>;
            }
        }
        impl reg::Dr for $adc::dr::DRrs {
            impl_read! {
                data -> adc1::dr::DATA_R;
            }
        }
    };
}

macro_rules! impl_cr2 {
    ($adc:ident) => {
        impl reg::Cr2R for $adc::cr2::CR2rs {
            impl_read! {
                adon -> adc1::cr2::ADON_R;
                cont -> adc1::cr2::CONT_R;
                cal -> adc1::cr2::CAL_R;
                rstcal -> adc1::cr2::RSTCAL_R;
                dma -> adc1::cr2::DMA_R;
                align -> adc1::cr2::ALIGN_R;
                jexttrig -> adc1::cr2::JEXTTRIG_R;
                exttrig -> adc1::cr2::EXTTRIG_R;
                jswstart -> adc1::cr2::JSWSTART_R;
                swstart -> adc1::cr2::SWSTART_R;
                tsvrefe -> adc1::cr2::TSVREFE_R;
            }
        }
        impl reg::Cr2W for $adc::cr2::CR2rs {
            impl_write! {
                adon -> adc1::cr2::ADON_W<Self>;
                cont -> adc1::cr2::CONT_W<Self>;
                cal -> adc1::cr2::CAL_W<Self>;
                rstcal -> adc1::cr2::RSTCAL_W<Self>;
                dma -> adc1::cr2::DMA_W<Self>;
                align -> adc1::cr2::ALIGN_W<Self>;
                jexttrig -> adc1::cr2::JEXTTRIG_W<Self>;
                exttrig -> adc1::cr2::EXTTRIG_W<Self>;
                jswstart -> adc1::cr2::JSWSTART_W<Self>;
                swstart -> adc1::cr2::SWSTART_W<Self>;
                tsvrefe -> adc1::cr2::TSVREFE_W<Self>;
            }
        }
        impl reg::ExtSelW for $adc::cr2::CR2rs {
            fn select_swstart(w: &mut W<Self>) -> &mut W<Self> {
                w.extsel().swstart()
            }
        }
    };
}

impl_ext!(adc1);
impl_cr2!(adc1);

#[cfg(not(feature = "stm32f100"))]
impl_ext!(adc2);
#[cfg(not(feature = "stm32f100"))]
impl_ext!(adc3);
#[cfg(not(feature = "stm32f100"))]
impl_cr2!(adc3);

# Source Verification Report

## Overall Assessment
- Sources reviewed: 58 (structural audit) + 8 (independent WebSearch spot-check)
- Verified (spot-check): 7 | Flagged: 1 CRITICAL + 3 tier/label adjustments | Rejected: 0
- Audit date: 2026-05-12

## CRITICAL finding — RESOLVED

Entry #15 had author-substitution hallucination. Bibliography attributed DOI 10.33012/navi.657 to "Štern, Džunda, Vosburgh." Actual authors: **Felux, M., Fol, P., Figuet, B., Waltert, M., & Olive, X.** (2024). DOI, title, methodology, and findings all match the real paper. Only author field was wrong. **Fixed in `01_bibliography.md`.**

## Tier adjustments recommended

- Entry #7 (Mitch et al. 2011): II → III (emission characterisation, no receiver-under-test)
- Entry #11 (Morong et al. 2019): II → III (propagation range study, no receiver mitigation tested)

Synthesis agent should apply these downgrades when weighting evidence.

## Predatory journal screening

All venues clean. Notes:
- **MDPI Sensors** (#20, #37, #38, #46, #48): indexed WoS/Scopus, not predatory, but editorial process variability noted. Findings should be corroborated where possible.
- **Inside GNSS** (#12, #44): correctly tagged gray literature.
- **IJET / Polish Academy of Sciences** (#11): legitimate, Scopus/DOAJ indexed.
- **IEEE CyberSA** (#47): legitimate workshop, lighter peer review than flagship venues.

## Conflict of Interest disclosures

### JRC (Joint Research Centre, EU Commission) — Low severity, institutional
9 of 58 entries (~16%): #9, #24, #25, #30, #31, #32, #42, #43, #27. JRC researchers (Borio, Gioia, Garcia-Peña) evaluate techniques they may recommend for EU standards. No financial COI. **Concentration risk**: the multi-layer mitigation framework (RIM > ANF > multi-constellation) appears across multiple JRC papers — synthesis stage must cross-check JRC-sourced quantitative claims against non-JRC sources where possible.

### DLR (German Aerospace Centre) — Low severity, institutional
Entries #17, #26. Government-funded research on government-developed receivers. Gao 2013 has flight-test data providing external anchor.

**Both COIs to be disclosed in paper's limitations / declaration section.**

## Currency assessment

All 10 pre-2015 anchors (Brennan 1959, Fante & Vaccaro 2000, Misra & Enge 2006, Groves 2013, Lashley 2010, Motella 2011, Mitch 2011, Bastide 2004, Musumeci & Dovis 2014, Gao 2013) PASS — actively cited by post-2020 work; foundational status confirmed.

## Hallucinated reference scan (full corpus)

- DOIs: all conform to `10.xxxx/...` pattern. ION proceedings without DOI (#7, #29, #40) confirmed via ION abstract IDs.
- Volume/issue numbers: all plausible.
- Titles: no suspiciously-perfect patterns.
- Author/venue mismatches: 1 CRITICAL (Entry #15) — fixed. 50 entries unspot-checked carry residual risk of similar author-substitution not detectable by DOI+title alone.

## Verification limitations

1. Spot-check covered 14% (8/58) of entries.
2. Automated DOI+title matching cannot catch author-substitution hallucinations where DOI resolves correctly.
3. ION pre-DOI entries rely on abstract-ID + PDF URL confirmation (PLAUSIBLE, not Tier 1 VERIFIED).
4. Semantic Scholar IDs cited by bibliography agent not independently re-queried.

## Recommendation to downstream synthesis agent

1. Apply Entry #7 + #11 tier downgrades.
2. Cross-check JRC concentration claims against independent (non-JRC) sources for any quantitative number reported >2× in the corpus.
3. Treat the Felux et al. 2024 correction as a worked example of why author-DOI alignment matters; surface this lesson in the paper's discussion of methodology limitations.
4. MDPI Sensors findings: prefer corroborating evidence.
